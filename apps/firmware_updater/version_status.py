"""Compare device versions against bundled approvals, never remote claims."""

import re
from html import escape

import bundled


def version_state(value, target):
    match = re.fullmatch(r"(\d+(?:\.\d+)+)(?:（[^）]*）)?", value.strip())
    if not target or not match:
        return "unknown", "版本待核实"
    current = tuple(map(int, match[1].split(".")))
    latest = tuple(map(int, target.split(".")))
    length = max(len(current), len(latest))
    current += (0,) * (length - len(current))
    latest += (0,) * (length - len(latest))
    if current == latest:
        return "current", "已是内置最新"
    if current < latest:
        return "outdated", f"请更新至 {target}"
    return "newer", "高于内置版本，请核实"


def device_summary(info, role, catalog):
    # Catalog defaults are independent of any manually selected rollback image.
    defaults = bundled.approved_defaults(catalog)

    def target(kind, model):
        path = defaults.get((kind, model))
        return next(
            (
                entry["version"]
                for entry in catalog["images"]
                if path
                and entry.get("file") == path
                and entry["kind"] == kind
                and entry["model"] == model
            ),
            None,
        )

    states = []
    colors = {
        "current": "#146448",
        "outdated": "#B42318",
        "unknown": "#626B78",
        "newer": "#815000",
    }

    def line(label, value, latest):
        state, message = version_state(value, latest)
        states.append(state)
        text = f"{label} {value} · {message}"
        return f'<span style="color:{colors[state]}">{escape(text)}</span>', text

    mcu_html, mcu_text = line("MCU", info["mcu"], target("mcu", role))
    if info.get("legacy_mcu_only"):
        text = "旧版 MCU 模式：只能更新 MCU，电机型号与版本无法读取；请核对实物 EL05。"
        brief = "旧版 MCU 模式 · 仅更新 MCU，请核对实物 EL05。"
        return f"{mcu_html}<br>{escape(brief)}", f"{mcu_text}\n{text}", "warning", text
    if role == "master":
        html, plain = f"主爪 · {mcu_html}", f"主爪 · {mcu_text}"
    else:
        model = info["model"]
        recorded = info["recorded"]
        prefix = f"型号 {model}（{'已记录' if recorded else '未配置'}）"
        motor_html, motor_text = line(
            "电机", info["motor"], target("motor", model) if recorded else None
        )
        can_problem = None
        if model == "RS00" and "can_id" in info:
            actual, expected = info["can_id"], info.get("can_id_expected")
            if expected is None:
                can_problem = "CAN ID 无法匹配 · 请核对型号记录和固件 SN"
                can_text, color = f"CAN ID {actual} · 待核实", colors["unknown"]
            elif actual != expected:
                can_problem = f"CAN ID 不符 · 开始更新后确认修正为 {expected}"
                can_text, color = (
                    f"CAN ID {actual} → {expected} · 待修正",
                    colors["outdated"],
                )
            else:
                can_text, color = f"CAN ID {actual} · 符合", colors["current"]
            motor_html += f' · <span style="color:{color}">{escape(can_text)}</span>'
            motor_text += f" · {can_text}"
        html = f"{escape(prefix)}　{mcu_html}<br>{motor_html}"
        plain = f"{prefix}  {mcu_text}\n{motor_text}"
    if "outdated" in states:
        tone, status = "error", "发现旧版固件 · 建议使用内置最新固件更新"
    elif all(state == "current" for state in states):
        tone, status = "success", "版本与内置最新一致 · 可按需重刷或选择其他版本"
    else:
        tone, status = "warning", "版本信息待核实 · 请核对后再选择固件"
    if role != "master" and can_problem:
        tone, status = "error", can_problem
    return html, plain, tone, status
