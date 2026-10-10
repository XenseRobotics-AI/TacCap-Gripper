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
    if role == "master":
        html, plain = f"主爪 · {mcu_html}", f"主爪 · {mcu_text}"
    else:
        model = info["model"]
        recorded = info["recorded"]
        prefix = f"型号 {model}（{'已记录' if recorded else '未配置'}）"
        motor_html, motor_text = line(
            "电机", info["motor"], target("motor", model) if recorded else None
        )
        html = f"{escape(prefix)}　{mcu_html}<br>{motor_html}"
        plain = f"{prefix}  {mcu_text}\n{motor_text}"
    if "outdated" in states:
        tone, status = "error", "发现旧版固件 · 建议使用内置最新固件更新"
    elif all(state == "current" for state in states):
        tone, status = "success", "版本与内置最新一致 · 可按需重刷或选择其他版本"
    else:
        tone, status = "warning", "版本信息待核实 · 请核对后再选择固件"
    return html, plain, tone, status
