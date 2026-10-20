"""Conservative readiness presentation; OS access is not an OTA handshake."""


def assess(report, identity=None, info=None):
    ports = report.get("ports", [])
    targets = [p for p in ports if not identity or p["serial"] == identity[1]]
    runtime = report.get("runtime", {}).get("ok", False)
    eligible = [
        p
        for p in targets
        if p.get("node_present") and p.get("read_write") and not p.get("owners")
    ]
    system_ready = bool(runtime and eligible)
    verified = bool(
        system_ready and identity and info and not report.get("connection_error")
    )
    configured = bool(verified and info.get("recorded"))
    checks = []

    def check(name, tone, value):
        checks.append(dict(name=name, tone=tone, value=value))

    check(
        "内置 SDK",
        "success" if runtime else "error",
        "加载正常" if runtime else "加载失败或未验证",
    )
    check(
        "USB 设备",
        "success" if targets else "warning",
        "已发现目标端口" if targets else "未发现目标夹爪",
    )
    check(
        "串口驱动",
        "success"
        if targets and all(p.get("driver", "unknown") != "unknown" for p in targets)
        else "warning",
        ", ".join(sorted({p.get("driver", "unknown") for p in targets}))
        if targets
        else "等待连接设备",
    )
    check(
        "串口读写权限",
        "success" if eligible else "error" if targets else "warning",
        "可访问目标端口"
        if eligible
        else "权限不足或端口被占用"
        if targets
        else "尚无端口可检查",
    )
    owners = [o for p in targets for o in p.get("owners", [])]
    check(
        "串口占用",
        "error" if owners else "neutral",
        "请关闭占用程序" if owners else "未检测到其他程序占用（非故障提示）",
    )
    risks = [
        i
        for i in report.get("issues", [])
        if i["code"]
        not in (
            "permission_denied",
            "node_missing",
            "tty_missing",
            "sdk_unavailable",
            "port_in_use",
        )
    ]
    check(
        "设备规则与会话",
        "warning" if risks else "success" if targets else "neutral",
        "存在待处理提示，展开详情查看"
        if risks
        else "未发现规则异常"
        if targets
        else "尚未验证",
    )
    check(
        "MCU 身份与通信",
        "error"
        if report.get("connection_error")
        else "success"
        if verified
        else "warning",
        "通信失败，请检查供电及日志"
        if report.get("connection_error")
        else "最近连接已验证；更新前再次校验"
        if verified
        else "尚未进行设备连接验证",
    )
    if not runtime:
        tone, title, badge = (
            "error",
            "环境异常：内置 SDK 未就绪，不能更新",
            "环境：异常",
        )
    elif not targets and not report.get("bridges"):
        tone, title, badge = "warning", "未连接夹爪，当前不能开始更新", "环境：未连接"
    elif not system_ready:
        tone, title, badge = "error", "串口条件未通过，当前不能开始更新", "环境：阻断"
    elif report.get("connection_error"):
        tone, title, badge = (
            "error",
            "系统条件通过，但设备通信失败，不能更新",
            "环境：通信失败",
        )
    elif not verified:
        tone, title, badge = "warning", "系统条件通过，尚需连接验证", "环境：待验证"
    elif not configured:
        tone, title, badge = "warning", "设备已连接，需先确认电机型号", "环境：待配置"
    elif risks or any(c["tone"] == "warning" for c in checks):
        tone, title, badge = (
            "warning",
            "设备已连接，但仍有环境提示待核查",
            "环境：有提示",
        )
    else:
        tone, title, badge = "success", "设备通信已确认，可进入更新预检", "环境：已就绪"
    next_step = (
        "返回主界面核对固件，再点击开始更新；仍会检查镜像、型号和设备状态。"
        if configured
        else "点击返回并连接验证，读取设备序列号与版本；此操作不刷写固件。"
        if system_ready and not report.get("connection_error")
        else "检查 USB；从爪还需接通 24 V。修复后自动复检，或点击重新检测。"
    )
    return dict(
        tone=tone,
        title=title,
        badge=badge,
        checks=checks,
        next_step=next_step,
        system_ready=system_ready,
        connection_verified=verified,
        configured=configured,
    )
