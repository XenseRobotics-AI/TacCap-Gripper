"""Presentation-only translations. Raw policy messages and device data stay intact."""

import re
from functools import lru_cache

_language = "zh"
CJK = re.compile(r"[\u3400-\u9fff]")

# Chinese remains the source language. Keep decisions based on the original
# messages; translate only at the GUI/log/dialog boundary, never in OTA policy.
WORDS = {
    "选择版本": "Select version",
    "历史版本": "historical",
    "内置最新": "bundled latest",
    "选择 .bin 文件…": "Browse for a .bin file...",
    "旧版不含 1.2.6 的控制串口接收错误恢复修复，仅用于回退对比。": "This older release lacks the 1.2.6 control-UART receive-error recovery fix. Use only for downgrade comparison testing.",
    "旧版不含 1.2.16 的电机回显帧过滤与后续修复。RS00 还缺少 1.2.15 的按 SN 确定开合方向修复；降级后必须检查实际开合方向，不要直接恢复生产运行。": "This release lacks the 1.2.16 motor-echo filtering and later fixes. For RS00, it also lacks the 1.2.15 SN-based direction fix. Check the physical opening/closing direction after downgrade; do not resume production directly.",
    "回滚只更换固件，不恢复旧配置；请先导出日志。完成后须重新验收，不保证旧版兼容现有配置。": "Downgrade replaces firmware only, not previous configuration. Export logs first. Revalidate before use; compatibility with existing configuration is not guaranteed.",
    "用户取消，未执行本项刷写；若此前已更新其他项目或切换协议，请先核对设备状态。": "Cancelled; this item was not flashed. If earlier items or the protocol changed, check the device state before use.",
    "MCU 版本在确认后发生变化，停止刷写，请刷新设备": "MCU version changed after confirmation. Flashing stopped; refresh the device.",
    "语言": "Language",
    "重复刷写": "reflash",
    "等待确认：": "Awaiting confirmation: ",
    "用户确认: ": "User confirmation: ",
    "检测到新连接的": "Newly connected: ",
    "，已切换设备类型": "; device filter updated",
    "简体中文": "Chinese",
    "全部夹爪": "All grippers",
    "设备类型": "Device type",
    "自动匹配固件": "Automatic firmware matching",
    "连接设备": "Device",
    "选择已连接的夹爪": "Select a connected gripper",
    "刷新": "Refresh",
    "正在自动扫描，请关闭其他控制程序。": "Scanning automatically. Close other control applications.",
    "夹爪固件": "Gripper firmware",
    "电机固件": "Motor firmware",
    "未选择": "Not selected",
    "选择文件": "Browse",
    "清除": "Clear",
    "型号未记录，请确认实物：": "Model not recorded. Check the physical motor:",
    "选择电机型号": "Select motor model",
    "设置型号": "Set model",
    "默认使用内置固件；可手动选择其他版本。": "Bundled firmware is selected by default. Browse to use another version.",
    "恢复默认": "Restore defaults",
    "请清空活动范围；仅在提示时断开 24 V，刷写中勿拔线。": "Clear the work area. Disconnect 24 V only when prompted. Do not unplug during flashing.",
    "等待选择设备和固件": "Select a device and firmware",
    "百分比仅表示当前固件传输；完成后还需重启和校验。": "Progress is for the current transfer only. Restart and verification follow.",
    "查看日志": "Show log",
    "收起日志": "Hide log",
    "导出日志": "Export log",
    "清空日志": "Clear log",
    "仅清空窗口显示；导出日志仍包含完整记录": "Clear the display only. Export still includes the full session.",
    "开始更新": "Start update",
    "方向与标定正常": "Calibration is correct",
    "仍要重刷": "Reflash anyway",
    "确认回滚": "Confirm downgrade",
    "确认继续": "Continue",
    "标定异常，停止": "Calibration incorrect: stop",
    "跳过此项": "Skip this item",
    "取消": "Cancel",
    "完成": "Done",
    "知道了": "OK",
    "正在查找匹配设备…": "Finding matching devices...",
    "主爪不适用": "Not applicable to a leader",
    "等待识别型号": "Waiting for motor identification",
    "型号未确认，不自动选择电机固件": "Motor firmware is not selected until the model is confirmed.",
    "主爪：刷写时勿拔 USB，完成后按提示拔插 USB。": "Leader: keep USB connected during flashing. Reconnect it only when prompted.",
    "内置": "bundled",
    "手动": "manual",
    "主爪 MCU": "Leader MCU",
    "从爪 MCU": "Follower MCU",
    "准备更新，请稍候": "Preparing update...",
    "重新扫描设备，请稍候…": "Rescanning devices...",
    "等待旧读取结束，随后重新扫描设备…": "Waiting for the previous read before rescanning...",
    "正在扫描设备…": "Scanning devices...",
    "正在连接并读取设备信息…": "Connecting and reading device information...",
    "请选择实物型号": "Select the physical motor model",
    "请根据电机标签确认 EL05 或 RS00，不要按固件默认值猜测。": "Check the motor label for EL05 or RS00. Do not guess from firmware defaults.",
    "确认电机型号": "Confirm motor model",
    "主爪固件": "Leader firmware",
    "主爪仅支持 MCU 更新，无需选择电机固件。": "Leaders support MCU updates only. No motor firmware is needed.",
    "选择固件": "Select firmware",
    "固件 (*.bin)": "Firmware (*.bin)",
    "镜像主/从角色与当前设备不匹配": "The image role does not match the selected leader/follower.",
    "文件校验失败": "File verification failed",
    "还差一步": "Selection required",
    "请先选择夹爪，并至少选择一个固件文件。": "Select a gripper and at least one firmware image.",
    "尚未就绪": "Not ready",
    "请等待连接；型号未记录时，先确认实物并设置型号。": "Wait for connection. If the model is unrecorded, check the physical motor and set its model first.",
    "请拔插主爪 USB": "Reconnect the leader USB",
    "请按步骤断开 24 V": "Power-cycle the 24 V supply",
    "需要你操作电源": "Power action required",
    "①  拔掉主爪 USB\n\n②  等待至少 2 秒\n\n③  重新插回 USB": "1. Disconnect the leader USB\n\n2. Wait at least 2 seconds\n\n3. Reconnect USB",
    "①  拔掉 24 V 电源\n\n②  等待至少 2 秒\n\n③  重新插回 24 V 电源": "1. Disconnect the 24 V supply\n\n2. Wait at least 2 seconds\n\n3. Reconnect the 24 V supply",
    "主爪通过 USB 供电，拔插 USB 即可。": "The leader is USB-powered. Disconnect and reconnect its USB cable.",
    "USB 保持连接。仅拔 USB 不算断电。": "Keep USB connected. Unplugging USB alone does not power-cycle a follower.",
    "正在检测 MCU 重启；随后仍需校验实际协议。": "Detecting the MCU restart. The actual motor protocol must still be verified.",
    "当前传输 %p%": "Current transfer: %p%",
    "继续处理确认结果": "Continuing after confirmation",
    "等待主爪 USB 重新插入（上限 180 秒）": "Waiting for leader USB reconnection (up to 180 seconds)",
    "等待用户断开 24 V 并重新上电（上限 180 秒）": "Waiting for you to power-cycle 24 V (up to 180 seconds)",
    "断电检测结束，继续检查结果": "Restart detection finished. Continuing verification.",
    "请选择设备查看版本": "Select a device to view its versions",
    "未发现匹配夹爪，等待连接；请关闭其他控制程序。": "No matching gripper found. Waiting for connection; close other control applications.",
    "等待设备 · 自动扫描中": "Waiting for a device / scanning automatically",
    "请选择目标设备": "Select the target device",
    "最新以本安装包内置审核固件为准；Flash 存档不等于实时验证。": "Latest refers to this package's approved firmware. A Flash record is not a live verification.",
    "读取设备失败": "Device read failed",
    "连接失败，请检查 USB、24 V、串口权限或设备占用，然后点击刷新。": "Connection failed. Check USB, 24 V power, serial permissions and other applications, then refresh.",
    "未连接 · 详情见日志": "Not connected / see log",
    "等待下一步": "Waiting for the next step",
    "已停止": "Stopped",
    "操作已停止 · 请查看详细日志": "Operation stopped / see the detailed log",
    "需要处理": "Action required",
    "更新完成 · 所选固件已核验": "Update complete / selected firmware verified",
    "更新完成": "Update complete",
    "日志 (*.log)": "Log (*.log)",
    "导出失败": "Export failed",
    "操作尚未结束": "Operation in progress",
    "请等待完成或超时，刷写中请勿拔线。": "Wait for completion or timeout. Do not disconnect cables during flashing.",
    "请以普通用户运行，禁止 sudo 启动升级界面。": "Run the updater as a normal user, not with sudo.",
    "中文字体加载失败，请重新安装升级工具": "Bundled font could not be loaded. Reinstall the updater.",
    "截图保存失败": "Screenshot could not be saved",
    "查找所选夹爪": "the selected gripper",
    "串口设备身份变化，停止": "Serial device identity changed. Stopping.",
    "版本读回时设备 SN 已变化，拒绝继续": "Device SN changed during version readback; refusing to continue",
    "心跳检查时设备 SN 已变化，拒绝继续": "Device SN changed during heartbeat check; refusing to continue",
    "MCU 重启后已请求恢复 OTA 蓝闪；后续状态查询不重置节奏": "OTA blue blinking requested after MCU restart; subsequent status queries will not reset its phase.",
    "已请求 OTA 蓝灯闪烁；刷写期间请勿拔线": "OTA blue blinking requested. Do not unplug during flashing.",
    "升级未完成：不会请求正常白灯，请以错误日志为准": "Update incomplete: normal white lighting will not be requested. Check the error log.",
    "设备身份变化，未恢复灯效": "Device identity changed; lighting was not restored",
    "固件与标定核验完成，已请求恢复正常白灯": "Firmware and calibration verified. Normal white lighting requested.",
    "读取设备身份、型号与固件版本（版本查询最多等待 3 秒）": "Reading identity, model and firmware versions (version query: up to 3 seconds)",
    "（Flash 存档，非实时）": " (Flash record, not live)",
    "Flash 存档，非实时": "Flash record, not live",
    "实时": "live",
    "（实时）": " (live)",
    "RS00 型号记录变化或未确认，停止 CAN ID 操作": "The RS00 model record changed or is unconfirmed. CAN ID operation stopped.",
    "无法从固件 SN 确认 RS00 左右手，拒绝猜测 CAN ID": "Cannot determine RS00 left/right side from the firmware SN. CAN ID will not be guessed.",
    "主爪不提供电机 CAN ID 设置": "Motor CAN ID configuration is not available for leaders",
    "确认修正 RS00 CAN ID": "Confirm RS00 CAN ID correction",
    "用户取消 CAN ID 修正，未开始固件刷写": "CAN ID correction cancelled. Firmware flashing has not started.",
    "确认期间 CAN ID 或型号状态变化，停止，请刷新设备": "CAN ID or model changed during confirmation. Stopping; refresh the device.",
    "等待设备空闲并失能电机，准备修正 CAN ID": "Waiting for idle and disabling the motor before CAN ID correction",
    "主爪不提供从爪电机型号设置": "Follower motor model configuration is not available for leaders",
    "不支持的电机型号": "Unsupported motor model",
    "设备已有型号记录，用户版不覆盖已有型号": "A model is already recorded. This user tool does not overwrite it.",
    "等待设备空闲，准备写入电机型号": "Waiting for idle before recording the motor model",
    "应用电机型号；重新上电可能触发自动标定": "Apply the motor model; power-on may trigger automatic calibration",
    "重启后电机型号读回不一致，停止更新": "Motor model readback mismatch after restart. Update stopped.",
    "准备断电检测：读取 MCU 启动时钟基线": "Preparing power-cycle detection: reading the MCU uptime baseline",
    "等待 MCU 时钟回退并连续增长": "Waiting for MCU uptime to reset and then increase",
    "未发现目标 USB 设备，请检查 USB 连接和设备供电": "Target USB device not found. Check USB connection and device power.",
    "主爪不能切换从爪电机协议": "A leader cannot switch a follower motor protocol",
    "等待标定结束并失能电机（单项等待上限 45 秒）": "Waiting for calibration and disabling the motor (up to 45 seconds per operation)",
    "需要切换并断电": "switch and power cycle required",
    "无需切换": "no switch required",
    "Private 协议未生效：是否恢复一次？": "Private protocol not applied: try one recovery?",
    "当前仍读回 MIT，电机尚未开始本次刷写。\\nMCU 重启不能证明电机已断电；也可能是切换或参数保存未生效。\\n确认后仅重新发送一次协议切换请求，再按弹窗断开整只夹爪的 24 V 至少 2 秒（USB 保持连接）。\\n不会重刷 MCU、不会绕过协议校验；再次失败将停止并保留日志。": "The motor still reports MIT. Motor flashing has not started.\nAn MCU restart does not prove the motor lost power; protocol switching or storage may also have failed.\nIf confirmed, one protocol-switch request will be sent again. Then disconnect the whole gripper's 24 V supply for at least 2 seconds, keeping USB connected.\nThe MCU will not be reflashed and protocol checks will not be bypassed. Another failure stops the operation and retains the log.",
    "用户取消协议恢复；未开始电机刷写，最后读回 MIT": "Protocol recovery cancelled. Motor flashing did not start; last protocol was MIT.",
    "用户确认一次协议恢复；重新核对设备、等待空闲后发送": "One protocol recovery confirmed. Rechecking identity and waiting for idle before sending.",
    "MCU 镜像与设备主/从角色不匹配": "The MCU image does not match the device's leader/follower role",
    "MCU 刷写：启动会话并传输固件，请勿拔线": "MCU flashing: starting the session and transferring firmware. Do not unplug.",
    "MCU 已提交，正在重启（可能短暂白灯/熄灭）；版本读回后恢复蓝闪（上限约 33 秒）": "MCU submitted and restarting (brief white/off lighting is possible). Blue blinking resumes after version readback (up to about 33 seconds).",
    "MCU 版本已读回；接下来仍须按提示完成物理断电，不代表升级全流程结束": "MCU version read back. A physical power cycle is still required when prompted; the full update is not finished.",
    "主爪不支持电机 OTA": "Motor OTA is not supported on leaders",
    "电机 OTA 预检：核对协议、型号与 UID": "Motor OTA preflight: checking protocol, model and UID",
    "电机刷写：等待握手并传输固件，请勿拔线": "Motor flashing: waiting for handshake and transferring firmware. Do not unplug.",
    "电机结束帧已确认；仍需断电、版本核验和恢复协议": "Motor end frame acknowledged. Power cycle, version verification and protocol restoration are still required.",
    "等待 Private 协议及电机实时版本（版本轮询上限 20 秒）": "Waiting for Private protocol and live motor version (version polling: up to 20 seconds)",
    "最终检查：等待 MIT 协议和自动标定完成": "Final checks: waiting for MIT protocol and automatic calibration",
    "等待最终标定": "final calibration",
    "最终标定未通过；固件可能已更新，请勿直接夹取": "Final calibration failed. Firmware may already be updated; do not operate the gripper.",
    "内置固件型号与清单不符": "Bundled firmware model does not match the catalog",
    " · 传输结束，仍需完成校验": " / transfer finished; verification still required",
    "传输结束，仍需完成校验": "transfer finished; verification still required",
    " · 暂无新传输回调，等待设备回执": " / no new transfer callback; waiting for a device response",
    "暂无新传输回调，等待设备回执": "no new transfer callback; waiting for a device response",
    "清除电机故障": "clearing the motor fault",
    "电机失能": "disabling the motor",
    "固件文件过大，拒绝读取": "Firmware file is too large; read refused",
    "文件不在内置校验清单中或类型错误，请向技术支持获取匹配的固件/工具。改名不能绕过校验。": "File is not in the approved catalog or has the wrong type. Obtain matching firmware/tool from support. Renaming cannot bypass verification.",
    "版本格式无效": "Invalid version format",
    "旧版不含 1.2.16 的电机回显帧过滤与后续修复。回滚后不要直接恢复生产运行。": "This older version lacks the motor echo-frame filtering and later fixes in 1.2.16. Do not resume production directly after downgrading.",
    "低于 EL05 支持下限 1.0.5.0.4；已知速度反馈异常，仅供必要的回退/对比测试，不宜直接夹取。": "Below the EL05 minimum supported version 1.0.5.0.4. Known velocity feedback issues; use only for necessary rollback/comparison tests, not normal gripping.",
    "刷写过程中不可断电；重新上电可能触发自动标定。": "Do not power off during flashing. Power-on may trigger automatic calibration.",
    "用户取消；如已切换协议，设备可能仍处于 Private，请联系支持或重新完成升级流程。": "Cancelled. If the protocol was switched, the device may still be in Private. Contact support or complete the update workflow again.",
    "设备 SN 未标明主/从角色，拒绝猜测": "Device SN does not identify a leader/follower role; refusing to guess",
    "主爪只允许主爪 MCU 镜像，不支持从爪或电机固件": "A leader accepts leader MCU images only, not follower or motor firmware",
    "开始主爪更新": "Start leader update",
    "用户取消，未修改设备": "Cancelled. Device unchanged.",
    "主爪 MCU 刷写完成，请拔插 USB": "Leader MCU flashing finished. Reconnect USB.",
    "主爪 MCU 实际版本不符": "Leader MCU version mismatch",
    "已核验主爪 MCU 为所选版本，跳过重复刷写；未修改设备。": "Leader MCU already matches the selected version. Reflash skipped; device unchanged.",
    "至少选择一种固件": "Select at least one firmware image",
    "请放下物体、清空活动范围并关闭其他控制程序。不会修改型号、方向或力矩参数。RS00 的 CAN ID 不符时将另行确认修正。": "Put down any object, clear the work area and close other control applications. Model, direction and torque settings will not be changed. Any RS00 CAN ID correction requires separate confirmation.",
    "更新后需拔插 USB。": "Reconnect USB when prompted after the update.",
    "更新预检：重新核对设备与固件": "Update preflight: rechecking device and firmware",
    "电机型号尚未配置，请联系技术支持；用户版不覆盖型号": "Motor model is not configured. Contact support; this user tool does not overwrite model records.",
    "此用户版仅支持从爪 MCU >= 1.2.14，旧设备请由技术支持初始化": "This user tool requires follower MCU >= 1.2.14. Older devices need initialization by support.",
    "电机型号与镜像不匹配，禁止刷写": "Motor model and image do not match. Flashing prohibited.",
    "不支持此 MCU 回滚目标": "Unsupported MCU downgrade target",
    "开始升级/回滚": "Start update / downgrade",
    "夹爪 MCU": "Gripper MCU",
    "正在刷写 MCU：不要断开电源或 USB": "Flashing MCU: do not disconnect power or USB",
    "MCU 刷写完成，必须硬重启": "MCU flashing complete; a physical power cycle is mandatory",
    "MCU 运行版本不符，停止后续步骤": "MCU running version mismatch. Subsequent steps stopped.",
    "正在刷写电机：不要断开电源或 USB": "Flashing motor: do not disconnect power or USB",
    "确认机械标定": "Confirm mechanical calibration",
    "请确认本次上电标定先闭合再张开、无异常撞击或卡滞。\n方向正确且活动正常吗？\n选择否将报告需要技术支持，不会自动改变方向。": "Confirm that power-on calibration closed the gripper first, then opened it, without abnormal impacts or jamming.\nAre direction and motion correct?\nSelecting No requests support; direction is not changed automatically.",
    "固件可能已写入，但机械标定未确认。不要运行夹取，请导出日志联系支持。": "Firmware may be written, but mechanical calibration is unconfirmed. Do not operate the gripper. Export the log and contact support.",
    "所选固件已逐项核验，电机已恢复 MIT，用户已确认标定。回滚设备请先完成对比验收，再恢复正常使用。": "Selected firmware verified, motor restored to MIT and calibration confirmed by the user. For a downgraded device, complete comparison acceptance before normal use.",
    "版本待核实": "version unverified",
    "已是内置最新": "matches bundled latest",
    "高于内置版本，请核实": "newer than bundled; check compatibility",
    "CAN ID 无法匹配 · 请核对型号记录和固件 SN": "CAN ID cannot be matched / check model record and firmware SN",
    "发现旧版固件 · 建议使用内置最新固件更新": "Older firmware detected / update to the bundled latest",
    "版本与内置最新一致 · 可按需重刷或选择其他版本": "Matches bundled latest / reflash or select another version if needed",
    "版本信息待核实 · 请核对后再选择固件": "Version unverified / check before selecting firmware",
    "请以普通用户注册应用菜单": "Register the application menu as a normal user",
    "此选项仅适用于 AppImage": "This option is only available for AppImage",
    "AppImage 菜单入口与图标已注册": "AppImage menu entry and icon registered",
    "主爪": "Leader",
    "从爪": "Follower",
    "电机": "Motor",
    "升级": "update",
    "回滚": "downgrade",
    "不修改": "unchanged",
    "不适用": "not applicable",
    "未知": "unknown",
    "已记录": "recorded",
    "未配置": "not configured",
    "包": "packs",
    "左": "Left",
    "右": "Right",
}

TEMPLATES = {
    "MCU 目标低于工具支持下限 {0}，请联系技术支持": "MCU target is below the tool's supported minimum {0}. Contact support.",
    "MCU 降级：{0} → {1}。只更换固件，不恢复旧配置；刷写前需再次确认。": "MCU downgrade: {0} -> {1}. Firmware only; previous configuration is not restored. Confirmation is required before flashing.",
    "等待电机协议 {0} 稳定超时，最后读回 {1}。停止后续步骤；确认断开的是 24 V（仅拔 USB 不够），保持至少 2 秒，再重新运行。若仍失败，保留日志排查协议切换。": "Timed out waiting for stable motor protocol {0}; last readback: {1}. Subsequent steps stopped. Disconnect 24 V (USB alone is insufficient), wait at least 2 seconds, then restart the workflow. If it still fails, retain the log for protocol diagnosis.",
    "{0} 等待设备空闲超时: {1}；停止后续升级，请检查标定状态及是否有其他控制程序运行。": "Timed out waiting for idle during {0}: {1}. Update stopped. Check calibration and other running control applications.",
    "{0} 失败: {1}": "{0} failed: {1}",
    "切换电机协议到 {0}": "switching motor protocol to {0}",
    "Xense TacCap 固件更新 · v{0} · build {1}{2}": "Xense TacCap Firmware Updater / v{0} / build {1}{2}",
    "警告：灯效 {0} 设置失败：{1}；不代表固件升级结果": "Warning: could not set lighting {0}: {1}. This does not determine the firmware result.",
    "警告：升级已核验，但正常灯效未恢复：{0}": "Warning: update verified, but normal lighting could not be restored: {0}",
    "CAN ID 读回不符：期望 {0}，实际 {1}；停止，不自动重写，请联系支持": "CAN ID mismatch: expected {0}, got {1}. Stopped; no automatic rewrite. Contact support.",
    "RS00 CAN ID 已核验：{0}（{1}爪）": "RS00 CAN ID verified: {0} ({1} gripper)",
    "CAN ID 已是 {0}，跳过写入和断电": "CAN ID is already {0}; write and power cycle skipped",
    "设备 {0}\n{1}爪 RS00：CAN ID {2} → {3}。\n目标由固件 SN 和 SDK 规则确定，不允许任意填写。\n写入后必须按提示断开整只从爪 24 V 至少 2 秒再上电；重新上电可能触发标定，请清空活动范围。取消将停止更新。": "Device {0}\n{1} RS00 gripper: CAN ID {2} -> {3}.\nThe target follows firmware SN and SDK rules; arbitrary values are not allowed.\nAfter writing, disconnect the entire follower's 24 V supply for at least 2 seconds when prompted, then reconnect. Power-on may trigger calibration; clear the work area. Cancel stops the update.",
    "CAN ID {0} 已请求写入，待断开 24 V 生效": "CAN ID {0} write requested; a 24 V power cycle is required",
    "应用 RS00 CAN ID {0}，重连后必须读回核验": "Apply RS00 CAN ID {0}; readback verification is required after reconnection",
    "写入电机型号：{0}": "Recording motor model: {0}",
    "断电检测基线：uptime={0} ms；等待物理断电，上限 180 秒": "Power-cycle baseline: uptime={0} ms; waiting for physical power cycle, up to 180 seconds",
    "MCU 时钟重置已确认：uptime={0} ms；这不能证明电机已断电，仍须校验实际协议": "MCU uptime reset confirmed: uptime={0} ms. This does not prove the motor lost power; actual protocol verification is still required.",
    "设备在线：uptime={0} ms；等待有效时钟回退及连续增长": "Device online: uptime={0} ms; waiting for a valid reset and subsequent increase",
    "设备暂时无法通信：{0}: {1}": "Device temporarily unreachable: {0}: {1}",
    "等待断电重启超时；{0}。未绕过校验，请导出日志": "Power-cycle wait timed out; {0}. Verification was not bypassed. Export the log.",
    "准备切换/核验 {0} 协议": "Preparing to switch / verify {0} protocol",
    "{0} 协议：{1}": "{0} protocol: {1}",
    "写入 {0} 协议启动配置": "Writing {0} protocol startup configuration",
    "应用 {0} 协议并重启电机": "Apply {0} protocol and restart the motor",
    "等待 {0} 协议连续两次读回一致（上限 20 秒）": "Waiting for two consecutive {0} protocol readbacks (up to 20 seconds)",
    "协议校验失败：{0}": "Protocol verification failed: {0}",
    "恢复前实际协议读回：{0}": "Actual protocol before recovery: {0}",
    "{0} 协议已核验": "{0} protocol verified",
    "传输 {0}%：{1}/{2} {3}": "Transfer {0}%: {1}/{2} {3}",
    "电机 UID: {0}；镜像 SHA256: {1}": "Motor UID: {0}; image SHA256: {1}",
    "保存已核验的电机版本记录：{0}": "Recording verified motor version: {0}",
    "已用时 {0}:{1} · 当前步骤 {2} 秒": "Elapsed {0}:{1} / current step {2} s",
    "已用时 {0}:{1}": "Elapsed {0}:{1}",
    "当前步骤 {0} 秒": "current step {0} s",
    "传输 {0}%（{1}/{2} {3}）": "transfer {0}% ({1}/{2} {3})",
    " · 传输 {0}%（{1}/{2} {3}）": " / transfer {0}% ({1}/{2} {3})",
    "进行中：{0}；{1}": "In progress: {0}; {1}",
    "{0}: {1}\n\n已停止，不会自动重试或回滚。设备可能已部分更新或仍处于 Private。请导出日志联系支持。": "{0}: {1}\n\nStopped. No automatic retry or rollback. The device may be partially updated or still in Private. Export the log and contact support.",
    "设备 {0}\n实物电机确认为 {1} 吗？\n\n选错型号会导致力矩/速度解释错误。请放下物体并清空活动范围。写入后必须按提示断开 24 V。已有行程配置不代表重新标定正确。": "Device {0}\nIs the physical motor definitely {1}?\n\nA wrong model causes incorrect torque/velocity interpretation. Put down any object and clear the work area. After writing, power-cycle 24 V when prompted. Existing travel settings do not prove calibration is correct.",
    "电机型号已写入并经重启核验：{0}": "Motor model recorded and verified after restart: {0}",
    "等待确认：{0}": "Awaiting confirmation: {0}",
    "用户确认: {0}": "User confirmation: {0}",
    "检测到新连接的{0}，已切换设备类型": "New {0} connected; device filter updated",
    "应用菜单图标注册失败（不影响升级）：{0}": "Application icon registration failed (updates are unaffected): {0}",
    "无法确认{0}当前版本，停止，不能凭存档推断电机实际版本": "Cannot confirm the current {0} version. Stopping; a stored record is not a live motor version.",
    "确认{0}重复刷写": "Confirm {0} reflash",
    "当前已是 {0}。通常无需重刷。\n选择跳过可减少刷写和断电步骤；需要重刷时请确认。": "Already running {0}. Reflashing is normally unnecessary.\nSkip to avoid extra flashing and power cycles, or confirm if a reflash is required.",
    "确认{0}升级": "Confirm {0} update",
    "确认{0}回滚": "Confirm {0} downgrade",
    "确认{0}{1}": "Confirm {0} {1}",
    "{0} → {1}\n\n{2}\n\n确认继续？": "{0} -> {1}\n\n{2}\n\nContinue?",
    "设备 {0}\nMCU {1} → {2}\n更新后需拔插 USB。": "Device {0}\nMCU {1} -> {2}\nReconnect USB when prompted after the update.",
    "主爪 MCU {0} 已更新并核验，无需电机标定。": "Leader MCU {0} updated and verified. Motor calibration is not required.",
    "设备 {0}\n型号 {1}\n": "Device {0}\nModel {1}\n",
    "设备 {0}": "Device {0}",
    "型号 {0}": "Model {0}",
    "{0} · {1}（{2}）": "{0} / {1} ({2})",
    "{0}（{1}）": "{0} ({1})",
    "电机: {0} → {1}\n": "Motor: {0} -> {1}\n",
    "\n请放下物体、清空活动范围并关闭其他控制程序。不会修改型号、方向或力矩参数。RS00 的 CAN ID 不符时将另行确认修正。": "\nPut down any object, clear the work area and close other control applications. Model, direction and torque settings will not be changed. Any RS00 CAN ID correction requires separate confirmation.",
    "跳过同版本 MCU：{0}；无需刷写或为它断电": "MCU already matches {0}: skipped, with no flashing or power cycle for this image",
    "MCU 实际版本已确认：{0}": "MCU running version confirmed: {0}",
    "跳过同版本电机：{0}；仍需恢复 MIT 协议": "Motor already matches {0}: flashing skipped; MIT must still be restored",
    "电机实时版本核验失败：目标 {0}，读回 {1}；未写入版本记录": "Live motor version verification failed: target {0}, readback {1}; no version record written",
    "电机实时版本已确认：{0}": "Live motor version confirmed: {0}",
    "请更新至 {0}": "update to {0}",
    "主爪 · {0}": "Leader / {0}",
    "型号 {0}（{1}）": "Model {0} ({1})",
    "CAN ID {0} · 待核实": "CAN ID {0} / unverified",
    "CAN ID 不符 · 开始更新后确认修正为 {0}": "CAN ID mismatch / confirm correction to {0} after starting",
    "CAN ID {0} → {1} · 待修正": "CAN ID {0} -> {1} / correction required",
    "CAN ID {0} · 符合": "CAN ID {0} / correct",
}


def language():
    return _language


def settings():
    from PySide6.QtCore import QSettings

    return QSettings("Xense", "TacCapFirmwareUpdater")


def set_language(code, *, persist=False, store=None):
    if code not in ("zh", "en"):
        raise ValueError("Unsupported UI language")
    global _language
    _language = code
    if persist:
        store = settings() if store is None else store
        store.setValue("ui/language", code)
        store.sync()


def initialize(override=None, *, store=None):
    store = settings() if store is None else store
    code = override or store.value("ui/language", "zh")
    set_language(code if code in ("zh", "en") else "zh")
    return language()


@lru_cache(maxsize=1)
def patterns():
    result = []
    for source, target in TEMPLATES.items():
        pieces = re.split(r"(\{\d+\})", source)
        expression = "".join(
            "(.*?)" if re.fullmatch(r"\{\d+\}", p) else re.escape(p) for p in pieces
        )
        result.append((re.compile(expression, re.S), target))
    return result


def _english(value, depth=0):
    if not CJK.search(value):
        return value
    if value in WORDS:
        return WORDS[value]
    stamp = re.match(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d )(.*)$", value, re.S)
    if stamp:
        return stamp[1] + _english(stamp[2], depth + 1)
    if depth < 8:
        for expression, target in patterns():
            match = expression.fullmatch(value)
            if match:
                return target.format(*(_english(v, depth + 1) for v in match.groups()))
    # Render rich summaries without changing HTML tags/attributes, numeric
    # versions, IDs or policy inputs. Translate composite plans line by line.
    if "<" in value and ">" in value:
        return "".join(
            part if part.startswith("<") else _english(part, depth + 1)
            for part in re.split(r"(<[^>]*>)", value)
        )
    if "\n" in value:
        return "\n".join(_english(line, depth + 1) for line in value.split("\n"))
    stripped = value.strip()
    if stripped != value:
        start = len(value) - len(value.lstrip())
        return (
            value[:start] + _english(stripped, depth + 1) + value[len(value.rstrip()) :]
        )
    if " · " in value:
        return " / ".join(_english(part, depth + 1) for part in value.split(" · "))
    original = value
    for source in sorted(WORDS, key=len, reverse=True):
        value = value.replace(source, WORDS[source])
    # Unexpected native diagnostics may contain text we cannot translate.
    # Keep every original code point losslessly rather than hiding the error
    # or presenting a misleading translation. Approved tool messages are
    # required by tests to avoid this fallback entirely.
    if CJK.search(value):
        return CJK.sub(lambda m: f"\\u{ord(m[0]):04x}", original)
    return value


def tr(value):
    value = str(value)
    if value.startswith("/"):
        return value  # User file paths are data, not translatable UI messages.
    if language() == "en":
        result = _english(value)
        return result.replace("\u3000", "&ensp;" if "<" in value else " / ")
    return value


def _bind(widget, method, value):
    bindings = getattr(widget, "_i18n_bindings", {})
    bindings[method] = value
    widget._i18n_bindings = bindings
    getattr(widget, method)(tr(value))


def set_text(widget, value):
    _bind(widget, "setText", value)


def set_tooltip(widget, value):
    _bind(widget, "setToolTip", value)


def set_placeholder(widget, value):
    _bind(widget, "setPlaceholderText", value)


def set_title(widget, value):
    _bind(widget, "setWindowTitle", value)


def set_format(widget, value):
    _bind(widget, "setFormat", value)


def add_items(widget, values):
    widget._i18n_items = list(values)
    widget.addItems([tr(v) for v in values])


def retranslate(root):
    from PySide6.QtCore import QObject

    for widget in [root, *root.findChildren(QObject)]:
        for method, value in getattr(widget, "_i18n_bindings", {}).items():
            getattr(widget, method)(tr(value))
        for index, value in enumerate(getattr(widget, "_i18n_items", [])):
            widget.setItemText(index, tr(value))
