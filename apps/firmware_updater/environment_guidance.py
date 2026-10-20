"""Bounded read-only probes and copyable, never executed, customer remedies."""

import os
import shlex
import subprocess
import time
from pathlib import Path

from host_diagnostics import RULE_NAME, read


def query(argv):
    """No shell, no sudo, no serial opens. Missing tools are unknown, not PASS."""
    try:
        result = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=1,
            check=False,
        )
        return result.returncode, result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return None, ""


def owners(node, *, proc=Path("/proc")):
    """Best effort; procfs permissions prevent proving a port is unoccupied."""
    found = []
    deadline = time.monotonic() + 0.15
    for process in proc.glob("[0-9]*"):
        if time.monotonic() >= deadline:
            break
        try:
            if int(process.name) == os.getpid():
                continue
            for fd in (process / "fd").iterdir():
                if fd.resolve() == Path(node).resolve():
                    found.append(
                        {"pid": int(process.name), "name": read(process / "comm")}
                    )
                    break
        except OSError:
            continue
    return found


def rule_text():
    here = Path(__file__).resolve().parent
    packaged = here / "serial.rules"
    return read(packaged) or read(
        here.parents[1] / "packaging/firmware_updater" / RULE_NAME
    )


def enrich(report, *, probe=query, find_owners=owners):
    """Keep the original schema/OS findings; add explicit evidence and actions."""
    issues = report["issues"]
    actions = []
    report["actions"] = actions
    report["os"] = {
        k: v.strip('"')
        for line in read(Path("/etc/os-release")).splitlines()
        if "=" in line
        for k, v in [line.split("=", 1)]
        if k in ("ID", "VERSION_ID")
    }
    report["ownership_scope"] = "best_effort_visible_processes_only"
    report["modemmanager"] = "not_checked"

    def issue(code, message):
        if not any(i["code"] == code for i in issues):
            issues.append({"code": code, "message": message})

    def action(message, commands=""):
        actions.append({"message": message, "commands": commands})

    if not report["runtime"]["ok"]:
        issue(
            "sdk_unavailable",
            "内置 SDK 无法加载或版本不匹配；请重新安装适配本平台的工具，不要安装 conda 或源码 SDK 来替代。",
        )
    codes = {i["code"] for i in issues}
    if "permission_denied" in codes:
        if report["dialout_configured"] and not report["dialout_active"]:
            action(
                "dialout 已配置：请完整注销桌面或 SSH 会话后重新登录；仅开新终端不够。"
            )
        elif not report["dialout_active"]:
            action(
                "仅在自己的终端执行权限配置，然后注销并重新登录；不要用 sudo 启动工具。",
                'sudo usermod -aG dialout "$USER"\nid',
            )
        else:
            action(
                "dialout 已生效但节点仍不可访问：请检查设备规则或联系管理员，不要 chmod 777。"
            )
    if report["bridges"] and not report["ports"]:
        action(
            "USB 已识别但没有 MCU 串口：把以下只读检查结果交给支持人员；不要盲目替换 Jetson 驱动。",
            "uname -m\nuname -r\nlsusb -d 1a86:\nlsusb -t\njournalctl -k -b --no-pager -n 80",
        )
    mm_code, mm_output = (None, "")
    if report["ports"]:
        mm_code, mm_output = probe(["systemctl", "is-active", "ModemManager.service"])
        report["modemmanager"] = mm_output if mm_code is not None else "unknown"
    needs_rules = bool(codes & {"permission_denied", "stable_path_missing"})
    # A single udev query keeps latency bounded regardless of how many devices
    # are attached. Remaining targets are explicitly not checked this time.
    for index, port in enumerate(report["ports"]):
        port["owners"] = find_owners(port["node"])
        port["modemmanager_ignore"] = "not_checked"
        if port["owners"]:
            issue(
                "port_in_use",
                "检测到进程持有 MCU 串口；请先正常关闭对应控制程序，再重新检测。工具不会强制结束进程。",
            )
        if index == 0 and port["node_present"]:
            code, properties = probe(
                ["udevadm", "info", "--query=property", "--name", port["node"]]
            )
            ignored = any(
                v in properties.splitlines()
                for v in ("ID_MM_DEVICE_IGNORE=1", "ID_MM_PORT_IGNORE=1")
            )
            port["modemmanager_ignore"] = ignored if code == 0 else "unknown"
            if mm_output == "active" and code == 0 and not ignored:
                issue(
                    "modemmanager_risk",
                    "ModemManager 正在运行且该 MCU 端口未标记忽略；可能干扰热插拔，但这不是已确认的失联原因。",
                )
                needs_rules = True
            if code != 0:
                issue(
                    "udev_check_unavailable",
                    "无法读取 udev 属性；此项未验证，请导出报告，不要视为驱动或权限已通过。",
                )
        if port["driver"] == "unknown":
            issue(
                "driver_unknown",
                "未读到 MCU 串口驱动绑定；已有节点不代表通信正常，请导出报告核查。",
            )
        if port["owners"]:
            action(
                "以下进程持有串口；请正常退出程序，不能据此直接强制结束系统服务。",
                "\n".join(f"ps -p {p['pid']} -o pid=,comm=" for p in port["owners"]),
            )
    if needs_rules:
        rules = rule_text()
        action(
            "推荐安装本平台 DEB 并在未刷写时重新插拔 USB。AppImage 用户也可审阅后复制以下规则配置；仅匹配夹爪 MCU，不停止系统服务。",
            (
                f"printf '%s\\n' {shlex.quote(rules)} | sudo tee /etc/udev/rules.d/{RULE_NAME} >/dev/null\nsudo udevadm control --reload-rules"
                if rules
                else ""
            ),
        )
        action(
            "应用规则后，在没有刷写任务时重新插拔该夹爪 USB，再点击重新检测。通常不需要重启 PC 或 Jetson。"
        )
    report["blocking"] = (
        not report["runtime"]["ok"]
        or bool(report["bridges"] and not report["ready_for_probe"])
        or bool(
            report["ports"]
            and all(p["owners"] or not p["read_write"] for p in report["ports"])
        )
    )


def commands(report):
    return "\n\n".join(
        a["commands"] for a in report.get("actions", []) if a["commands"]
    )
