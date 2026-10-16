"""OS prerequisites without opening serial ports or sending firmware commands."""

import grp
import os
import platform
import pwd
import stat
from pathlib import Path

RULE_NAME = "69-taccap-firmware-updater.rules"


def read(path):
    try:
        return path.read_text().strip()
    except OSError:
        return ""


def is_bridge(path):
    return (
        read(path / "idVendor").lower() == "1a86"
        and read(path / "product") == "USB Dual Serial"
    )


def collect(
    *, sys_root=Path("/sys"), dev_root=Path("/dev"), access=None, char_node=None
):
    access = access or (lambda p: os.access(p, os.R_OK | os.W_OK, effective_ids=True))
    char_node = char_node or (lambda p: p.exists() and stat.S_ISCHR(p.stat().st_mode))
    uid = os.geteuid()
    user = pwd.getpwuid(uid).pw_name
    try:
        dialout = grp.getgrnam("dialout")
        active = dialout.gr_gid in os.getgroups() or dialout.gr_gid == os.getegid()
        configured = (
            user in dialout.gr_mem or pwd.getpwuid(uid).pw_gid == dialout.gr_gid
        )
    except KeyError:
        active = configured = False
    bridges = [
        {"usb": path.name, "serial": read(path / "serial")}
        for path in sorted((sys_root / "bus/usb/devices").glob("*"))
        if is_bridge(path)
    ]
    ports = []
    for tty in sorted((sys_root / "class/tty").glob("tty*")):
        if not tty.name.startswith(("ttyACM", "ttyUSB")):
            continue
        path = tty.resolve()
        parents = (path, *path.parents)
        usb = next((p for p in parents if is_bridge(p)), None)
        interface = next(
            (p for p in parents if (p / "bInterfaceNumber").is_file()), None
        )
        if (
            usb is None
            or interface is None
            or read(interface / "bInterfaceNumber") != "02"
        ):
            continue
        serial = read(usb / "serial")
        node = dev_root / tty.name
        alias = dev_root / "serial/by-id" / f"usb-1a86_USB_Dual_Serial_{serial}-if02"
        try:
            present = char_node(node)
            writable = present and access(node)
            stable = (
                present and alias.is_symlink() and alias.resolve() == node.resolve()
            )
        except OSError:
            present = writable = stable = False
        ports.append(
            {
                "node": str(node),
                "serial": serial,
                "driver": (interface / "driver").resolve().name
                if (interface / "driver").is_symlink()
                else "unknown",
                "node_present": present,
                "read_write": writable,
                "stable_path": str(alias),
                "stable_present": stable,
            }
        )
    issues = []
    if bridges and not ports:
        issues.append(
            {
                "code": "tty_missing",
                "message": "USB 已识别，但 MCU 串口节点未就绪；请检查 USB 串口驱动绑定和连接。安装 conda 不能代替内核驱动。",
            }
        )
    for port in ports:
        if not port["node_present"]:
            issues.append(
                {
                    "code": "node_missing",
                    "message": "MCU 串口节点不存在；请重新连接 USB 并检查驱动。",
                }
            )
        elif not port["read_write"]:
            issues.append(
                {
                    "code": "permission_denied",
                    "message": "当前用户没有 MCU 串口读写权限。请安装带串口规则的 DEB 后重新插拔 USB；远程/无桌面会话可加入 dialout 组并注销重新登录。",
                }
            )
        if not port["stable_present"]:
            issues.append(
                {
                    "code": "stable_path_missing",
                    "message": "缺少 SDK 所需的 /dev/serial/by-id MCU 链接。请安装带设备规则的 DEB 后重新插拔 USB。",
                }
            )
    if configured and not active:
        issues.append(
            {
                "code": "login_required",
                "message": "dialout 已配置但当前会话尚未生效；请注销重新登录，不要以 sudo 启动界面。",
            }
        )
    return {
        "schema": 1,
        "architecture": platform.machine(),
        "kernel": platform.release(),
        "user": user,
        "uid": uid,
        "dialout_active": active,
        "dialout_configured": configured,
        "bridges": bridges,
        "ports": ports,
        "ready_for_probe": any(p["read_write"] for p in ports),
        "issues": issues,
        "scope": "只检查系统环境；未打开串口、读取固件 SN 或验证 OTA 通信。",
    }


def assert_access(chip_serial=None, report=None):
    report = report if report is not None else collect()
    ports = report["ports"]
    if chip_serial:
        ports = [p for p in ports if p["serial"] == chip_serial]
    if any(p["read_write"] for p in ports):
        return report
    if ports or (not chip_serial and report["bridges"]):
        message = "\n".join(
            dict.fromkeys(issue["message"] for issue in report["issues"])
        )
        raise RuntimeError(message or "USB 串口环境未就绪，请运行环境诊断。")
    return report


def diagnostic_report():
    report = collect()
    try:
        import xense.taccap as sdk
        from xense.taccap import _taccap_native as native

        report["runtime"] = {
            "ok": True,
            "sdk": sdk.__version__,
            "native": native.__file__,
        }
    except Exception as exc:
        report["runtime"] = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    return report


def summary(report):
    lines = [
        "固件工具自带 Python 和 SDK，不需要安装 README 中的开发环境。",
        f"平台：{report['architecture']}  内核：{report['kernel']}",
        f"当前用户：{report['user']}  dialout 当前会话：{report['dialout_active']}",
    ]
    if "runtime" in report:
        lines.append(
            "内置 SDK 加载："
            + ("正常" if report["runtime"]["ok"] else report["runtime"]["error"])
        )
    lines.extend(
        f"{p['node']}  驱动 {p['driver']}  读写 {p['read_write']}  稳定链接 {p['stable_present']}"
        for p in report["ports"]
    )
    lines.extend(issue["message"] for issue in report["issues"])
    if not report["bridges"] and not report["ports"]:
        lines.append("尚未发现匹配的 USB 串口桥；请检查 USB 线缆和端口。")
    if report["ready_for_probe"]:
        lines.append(
            "USB 与访问权限检查通过；不代表 MCU 已应答。若仍 GetSn 超时，请检查 24 V、占用程序和通信日志，不要反复安装 SDK。"
        )
    lines.append(report["scope"])
    return "\n".join(lines)
