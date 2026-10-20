# Ubuntu 用户版固件升级与回滚

提供 Ubuntu 24.04 amd64 构建和 Ubuntu 22.04 ARM64 / Jetson 构建入口（Jetson 实机验收须单独完成），面向普通用户，支持主爪 MCU 更新，以及从爪 MCU 与 EL05/RS00 电机固件单独或组合更新。
回滚指重新刷入指定的旧版本镜像，不是恢复配置备份，也不是保证升级失败后可自动恢复。
首版需要现场硬件验收后再向最终用户分发；模拟测试不能证明断电时序和机械安全。

## 工具范围

PC 与 Jetson 提供相同的固件维护功能：主/从爪更新、固件选择与升降级、中英文、日志和环境诊断。
用户版不提供电机控制器、参数调节或滑条运动测试；更新完成后需按现场流程另做功能验收。
SDK 控制器和独立示例仍保留在 SDK 项目中，不通过固件工具开放。
两平台的 Python/Qt/原生 SDK 分别构建，不能复制或改名二进制来跨平台运行。

## 独立运行与环境诊断

安装包自带匹配的 Python、Qt、SDK，**不需要按项目 README 安装 mamba/conda 或源码 SDK**。
源码 SDK 的开发环境与用户版工具的操作系统前置条件不是一回事。
README 同时包含开发环境安装和 `dialout` 权限配置，执行整个流程后恢复不能单独证明缺少 SDK。
原始 `5dd8cb5` ARM64 包已在无完整 SDK/conda 的隔离 ARM64 用户空间通过 SDK 加载自检；该结果不代表真机 USB/OTA 验收。

新版 DEB 声明系统运行库与 udev 依赖，并安装 `69-taccap-firmware-updater.rules`：
仅匹配 `1a86 / USB Dual Serial / interface 02`，设置 `dialout + 0660 + uaccess`，补充稳定串口链接，并标记此 MCU 端口供 ModemManager 忽略（不停止服务、不影响其他串口）。
安装只重载规则，不批量触发设备、不自动加组、不装驱动或改 CUDA，不停止 ModemManager，不运行 root UI。
**未刷写时**退出工具并重新插拔该夹爪 USB，使规则应用；常规桌面会话通常无需重启 Jetson。
远程/无活动桌面会话仍需手动配置 `dialout` 并重新登录。AppImage 不会自动向系统安装 udev 规则。

点击“环境诊断”，结果也会写入导出日志；或使用不发送夹爪命令的 CLI：

启动及刷新扫描前会先检查系统环境；发现异常时提示具体原因，并提供“复制命令 / 重新检测 / 导出报告”。
环境诊断顶部与主界面入口显示明确结论：绿色表示对应检查通过，黄色表示待处理或尚未验证，红色表示阻断。
SDK 加载通过不等于能够更新：未连接夹爪时明确显示不能开始更新；端口可访问后仍需正常连接、读取 MCU 身份和版本。
只有设备通信已确认、型号已记录且没有环境提示时，才显示绿色“可进入更新预检”，不承诺刷写必然成功；镜像/角色/版本/设备状态仍由更新预检校验。
诊断窗口打开时每 3 秒自动做只读环境复检；不会自动连接或刷写。点击“返回并连接验证”回主界面扫描、读取身份，不写固件。
分项检查采用颜色加文字，完整诊断详情默认折叠，展开或收起会同步收缩窗口，避免大片空白。
命令不会自动执行；请先审阅，再在自己的终端操作。加入 dialout 后须完整注销再登录；仅开新终端不够。
AppImage 不安装系统规则，诊断会按实际问题提供规则配置指令。应用规则后，在没有刷写任务时重新插拔 USB；通常不必重启 PC 或 Jetson。
串口占用只检查当前用户可见的进程，无发现不等于无人占用；ModemManager 运行仅是风险，不能据此认定通信失败的根因。
诊断不打开串口、不校验固件身份或 OTA；USB/权限正常后仍需正常扫描握手。内置 SDK 加载失败应换用正确架构的工具，不应依赖客户另装开发环境。

```bash
taccap-firmware-updater --diagnose
./taccap-firmware-updater_0.1.0_aarch64.AppImage --appimage-extract-and-run --diagnose
```

| 诊断结果 | 含义与处理 |
| --- | --- |
| 内置 SDK 加载失败 | 检查包内库、系统依赖和架构；不要求安装开发环境来掩盖打包问题 |
| USB 桥未发现 | 检查 USB 线缆、端口和设备连接 |
| USB 已发现，无 MCU tty | 检查内核 USB 串口驱动绑定；安装 Python SDK 不能替代驱动 |
| tty 无读写权限 | 应用 DEB 规则、重新插拔 USB；必要时配置 dialout 并重新登录 |
| 缺少 by-id 链接 | 新规则补稳定链接；新版 SDK 可回退到原始 tty，但仍需验证固件身份 |
| USB/权限正常，GetSn 仍超时 | 属于 MCU 通信问题，核对 24 V、其他占用程序并导出日志，不能归因于缺少 SDK |

## Jetson AGX Orin / Ubuntu 22.04 ARM64

不能在 Jetson 原生运行现有的 x86_64 / amd64 安装包。赋予执行权限或修改文件名都不能转换架构。
本目标固定为 Ubuntu 22.04、aarch64、glibc 2.35；Qt 使用 PySide6 6.7.3（ARM64 wheel 为 manylinux_2_31），
不复用 Ubuntu 24.04 的 Qt/SDK 二进制，不安装或替换 CUDA、JetPack、NVIDIA 驱动。

也可在 Docker Engine Community 中使用 Ubuntu 22.04 ARM64 容器进行构建。x86_64 主机须先经管理员
许可注册 ARM64 QEMU/binfmt 执行支持，否则容器启动会报 exec format error。只读挂载源码，
在容器内独立 checkout 中以普通用户运行构建入口，不覆盖宿主机 SDK。Docker 和模拟器只用于构建，
客户运行安装包不需要它们。模拟环境自检不代表已完成 Jetson 真机的图形驱动、串口及 OTA 验收。

若默认 binfmt 不识别 AppImage 的特殊 ELF 文件头，在 SDK 编译后使用
`build/jetson-updater/venv/bin/python packaging/firmware_updater/build_all.py --extract-appimages`
先校验并解包后执行其中程序；构建环境需安装 `squashfs-tools`，无需新增主机注册规则。
此方式验证 AppImage 内容，不代表已在真机验证 FUSE 挂载。ARM64 deb 使用 gzip 压缩，缩短模拟构建耗时。

### 在专用 ARM64 构建机或 Jetson 上构建

使用本分支的独立干净 checkout，不要复用含 x86_64 扩展或其他 Python ABI 扩展的源码目录。
确认 firmware/motor/ 中的受审镜像齐全：EL05 默认镜像已随仓库提供，RS00 仍需本地提供。先提交本次源码修改。
以下依赖只需在构建机安装；最终用户不需要编译环境或 conda：

```bash
uname -m
cat /etc/os-release
sudo apt update
sudo apt install build-essential cmake ninja-build git python3-dev python3-venv libopencv-dev libspdlog-dev patchelf binutils fonts-noto-cjk libxcb-cursor0 libxkbcommon-x11-0 libsm6 libice6 libxcb-shape0 libxcb-icccm4 libxcb-keysyms1 libgl1 libegl1
python3 packaging/firmware_updater/build_jetson.py --jobs 2
```

入口在 build/jetson-updater/ 中创建隔离 venv 并重编译当前 SDK，默认两路编译以控制内存。
不会联网更新固件，构建依赖下载需要网络。缺失/错误架构扩展、固件哈希不符或未提交源码均不能作为完成的发布包。
构建成功后 dist/ 应包含：

- taccap-firmware-updater_0.1.0_arm64.deb
- taccap-firmware-updater_0.1.0_arm64.run
- taccap-firmware-updater_0.1.0_aarch64.AppImage

与 amd64 产物使用不同文件名，不互相覆盖。构建自检不操作夹爪；Qt X11 界面、串口权限、
设备发现、真实断电与 OTA 必须在目标 Jetson 上另做验收，不能以 x86_64 单元测试替代。

### 用户端启动（取得已验收的 ARM64 包后）

优先使用 deb，由 apt 补齐系统图形库：

```bash
sudo apt install ./taccap-firmware-updater_0.1.0_arm64.deb
sudo usermod -aG dialout "$USER"
```

注销并重新登录，以普通用户从应用菜单启动。便携包仍需要系统图形库和串口权限：

```bash
chmod +x taccap-firmware-updater_0.1.0_aarch64.AppImage
./taccap-firmware-updater_0.1.0_aarch64.AppImage --appimage-extract-and-run --self-test
./taccap-firmware-updater_0.1.0_aarch64.AppImage --appimage-extract-and-run --ui-smoke-test
```

extract-and-run 可避免依赖 FUSE。若文件管理器显示“没有应用可打开 AppImage”，先检查架构与执行权限，
不要到软件商店寻找能把 x86_64 应用变成 ARM64 的打开程序。禁止 sudo 运行界面。

## 安装与权限

```bash
sudo apt install ./taccap-firmware-updater_0.1.0_amd64.deb
sudo usermod -aG dialout "$USER"
```

加入组后注销并重新登录。仅安装和配置串口权限需要管理员；从应用菜单打开
“Xense TacCap 固件更新”，**禁止 sudo 启动 UI，禁止 chmod 777 串口**。
应用菜单、窗口和任务栏使用用户提供的夹爪图片图标。
软件包内含冻结的 Python 解释器、Qt、Noto 中文字体与匹配的原生 SDK，不使用用户 conda 或用户
site-packages。启动器清除 Python/动态库路径注入变量；系统图形库由 apt 依赖提供。
在 Qt 初始化字体之前，Linux 启动入口使用 `/etc/fonts/fonts.conf` 和 `/etc/fonts`
作为进程内 Fontconfig 配置，避免冻结库寻找构建机的 conda 路径。显式设置的
`FONTCONFIG_FILE` 或 `FONTCONFIG_PATH` 保持不变；不会修改系统字体配置或隐藏加载错误。
不自动添加用户到组、不更改全局串口权限。随包包含主爪 MCU 1.2.6、从爪 MCU 1.2.16、
EL05 1.0.5.0.4、RS00 0.0.3.32 四份默认镜像，不内置所有历史二进制；构建和加载时均按 SHA-256 校验。
这是本次仓库/本地审核清单的版本快照，不会联网自动下载更高版本。

## 用户操作

1. 放下物体、清空活动范围，关闭控制台/其他上位机后主爪连接 USB；从爪连接 USB 和 24 V。
2. 启动后自动扫描；没有设备时每 5 秒重试，可筛选全部/主爪/从爪，只有一只匹配设备时自动连接读取信息，
   多只时必须选择目标。拔插或更换夹爪后点击“刷新”重新发现设备；若旧类型筛选下没有匹配项，
   且当前只连接一只另一类型的夹爪，手动刷新会跟随其主/从角色切换筛选并读取信息。
   读取中刷新会清掉旧信息，等待旧读取释放串口后重新扫描，旧任务的结果不会覆盖新设备。
   连接失败后清除旧设备列表，空闲时继续每 5 秒扫描，也可手动刷新。刷写/型号写入期间仍禁止刷新或切换目标。连接仅在操作时
   短暂打开串口，读完释放；不会自动刷写。版本查询可能使旧电机停止，不能与夹取同时进行。
3. 根据 SN 的 `m`/`s` 后缀匹配主/从角色（不按左右或端口号猜测），默认选择内置最新 MCU 和与已记录型号对应的电机固件。
   MCU 行点击“选择版本”，只展示该角色的内置最新版本与手动选择 `.bin` 的入口，不列出历史版本。
   主爪历史目标 1.2.5、从爪 1.2.14/1.2.15 仍可通过手动选择已审核的文件回滚；校验清单不等于内置二进制。文件内容必须匹配 SHA-256，选错文件或取消不会覆盖原选择。
   清除某项表示不更新，可用“恢复默认”重新选用最新镜像。只降级 MCU 时请清除电机固件，避免不必要的电机查询和协议切换。
   手动选择仍须匹配包内 SHA-256 校验清单，不能靠改名通过。
   型号未记录时显示设置入口：根据实物标签选择 EL05/RS00，确认后写入、按弹窗断开
   24 V，重启读回一致才启用对应默认镜像。不能按 MCU 的编译默认型号猜测；
   用户版不覆盖已有型号，也不修改方向或限矩。
4. 确认计划；每次刷写前再次确认当前版本到目标版本的升级/回滚/同版本重刷。
   EL05 1.0.5.0.2 有已知速度反馈异常，低于支持下限，风险弹窗不可跳过。
   弹框按钮按操作区分：红色表示回滚或标定异常停止，黄色表示刷写/重刷及配置变更，
   绿色表示完成、标定正常或跳过重复刷写；取消保持中性灰色。文字和图标仍明确说明操作，
   不依赖颜色判断。回车、Esc 或关闭确认框默认取消/跳过，不会确认写入。
5. 主爪刷写完成后按提示拔插 USB，不涉及电机协议或电机标定。从爪**只在断电弹窗出现时**拔掉所选夹爪 24 V 至少 2 秒，再插回，USB 保持连接。
   弹窗没有“已完成”跳过按钮；检测心跳回退并核对协议后继续。心跳回退也可能由
   软复位引起，不能证明物理断电，用户仍须实际拔掉 24 V。
6. 写入过程中不要拔线，不可切换设备、重复开始或关闭窗口。恢复 MIT 可能自动标定。
   最后确认先闭合再张开、无卡滞；不正确则停止，UI 不自动改方向/力矩。
7. 更新或设置型号时自动展开日志；协议/标定/重启等待显示动态进度条、耗时和
   每 5 秒的当前步骤日志。刷写显示 SDK 回调的实际传输百分比及字节数/包数，
   100% 仅表示当前镜像传输结束，不代表整个升级已完成。暂无新回调时会明确提示。
   从爪实际开始 MCU 或电机刷写前，工具显式请求固件的 OTA 蓝闪预设，并留出首帧刷新时间；
   同一次运行期间反复打开串口或查询状态不重发蓝闪，避免重置闪烁相位；MCU 刷写提交
   或已有重启检测确认后，才在下一次身份核验通过的连接上恢复一次蓝闪，不额外查询启动时钟。
   MCU 软重启后，复用首次版本读回且 SN 核验通过的连接立即恢复蓝闪，不再拖到人工断电结束；
   保留原有 3 秒重启缓冲和后续物理断电要求。软重启期间可能短暂白灯或熄灭，界面会明确提示。
   全部版本和标定确认完成后才请求恢复正常白灯。蓝闪仍由 MCU 调度，工具侧不能保证
   刷写高负载期间严格保持亮 300 ms、灭 300 ms；断电与重启会中断灯效。
   故障时在仍可通信的当前连接上请求红闪，不为恢复白灯而重连或自动重试升级。
   断电期间无法控制灯效，重新上电后固件可能先显示启动灯语；灯效命令的 ACK 不能证明
   实物已正确发光。灯效失败会记录警告且不覆盖原始升级结果，现场仍须观察验证。
   故障时导出日志联系支持。此前某项固件可能已经更新、协议可能仍为 Private，
   不要直接运行夹取。没有自动重试、自动回滚或自动运动恢复。

### 界面语言 / Interface language

- 顶部“语言 / Language”选择 `English` 或 `简体中文`，空闲时即时生效并记住当前用户的选择，无需重装。升级、读取设备或型号设置期间禁用切换，避免打断流程。
- 主从爪界面、版本状态、确认/回滚警告、断电步骤、进度、日志显示及导出均随语言切换。设备 SN、版本、CAN ID 和实际文件路径保持原值；切换不会访问设备或更改刷写判断。
- 保留原始日志供重新显示/导出；清空显示后切换语言不会重新露出已清空的记录。未知的第三方中文诊断在英文模式下以无损 Unicode 转义保留，不猜测其含义；工具自身提示必须有英文译文。
- 临时指定启动语言（不覆盖保存的选择）：`taccap-firmware-updater --language en` 或 `--language zh`。AppImage 接受相同参数。

Choose **English** in the **Language** selector for an English interface. The choice applies immediately while idle and is saved per user. Switching is disabled during operations. Confirmations keep the same safe defaults and color meanings; no firmware, model, direction, torque or OTA policy changes are made by switching languages.

MCU 升级先于电机更新；MCU 回滚放在电机核验之后。电机必须实时读回目标版本才
写 Flash 版本记录，存档不能充当升级成功证据。从爪 MCU 当前/目标版本均须
>= 1.2.14，主爪目标须 >= 1.2.5。升级和降级均先确认 MCU 变化，再执行任何电机/CAN ID 写入；
实际 MCU 降级写入仍在电机核验后，写前再次读取当前 MCU，确认后发生版本变化则停止，不重用旧确认。
版本比较统一将零 build 后缀归一化，例如 `1.2.16` 与 `1.2.16.0` 是同一版本，不误判为降级或读回失败。
降级只更换固件，不恢复旧配置，不保证旧固件兼容现有配置；确认弹框为红色且默认取消。
从爪 1.2.14 缺少 RS00 在 1.2.15 引入的按 SN 定方向修复；1.2.15 缺少 1.2.16 的电机回显帧过滤修复。
回滚后必须人工核对实际开合方向并重新验收，不能直接恢复生产运行。
未记录型号可通过上述入口初始化；RS00 从爪在更新预检中按固件 SN 调用 SDK 规则核对 CAN ID（左 17、右 18）。
不符时独立确认，等待标定结束并失能后写入，引导断开 24 V 至少 2 秒，重连读回成功才继续 OTA；
已匹配则不写入、不额外断电。扫描本身不写设备；EL05/主爪不套用此规则，RS00 型号未记录或 SN 侧别不明时拒绝写入。
确认期间状态变化、断电超时或读回不符均停止，不自动重写；完成升级后再次核验 CAN ID。
力矩/方向设置仍由技术支持完成。

## 镜像清单与边界

`apps/firmware_updater/catalog.json` 是随应用发布、普通用户不可写的内容校验清单：
主爪 MCU 1.2.6 与 1.2.5、从爪 MCU 1.2.16/1.2.15/1.2.14 来自本仓库历史；电机 EL05 1.0.5.0.2、1.0.5.0.4 和
RS00 0.0.3.32 来自本次工作区中用户提供的文件。哈希确保选择的文件与开发时
检查的内容相同，**不等于厂商数字签名或厂商认证**。新固件由维护者审核型号、
来源、版本和兼容性后更新清单与安装包，不开放用户自行导入信任清单。
未将设备当前固件读取为备份，也不保证损坏的 bootloader 可由此工具恢复。

旧 MCU 的完整提交和历史文件路径固定在清单 `history` 字段，供维护者核对来源。
构建器只打包四份最新默认镜像，不从 Git 恢复历史文件；运行也不调用 Git 或下载固件。
历史版本需由支持人员提供对应的已审核文件，任意改名、篡改、未知版本和低于兼容下限的文件均不能绕过检查。

### 升降级验证边界

| 范围 | 软件回归要求 | 现场验收要求 |
| --- | --- | --- |
| 主爪 MCU 1.2.5/1.2.6 | 升级、降级、同版本确认；USB 重启后读回目标版本 | 目标设备实际刷写、USB 拔插与功能测试 |
| 从爪 MCU 1.2.14/1.2.15/1.2.16 | 升降级顺序、取消不写入、确认后版本变化停止、读回不符失败 | 24 V 断电、协议恢复、标定方向和行程、夹取验收 |
| 电机 | 镜像型号/哈希、升降级确认、实时版本读回、与 MCU 组合顺序 | 对应电机的真实刷写、反馈与安全验收 |

模拟与离线测试不能保证任意历史固件都兼容当前硬件/配置，也不替代真实升降级验收；只支持清单内经过审核且满足兼容下限的镜像。

## 构建与验证

在 Ubuntu 24.04 amd64 上构建当前 checkout 的 SDK；构建 Python 需有 PySide6 6.8.3、
PyInstaller 6.22.3 及匹配的 Python ABI，系统需有 patchelf 和 fonts-noto-cjk。构建环境可以使用 conda，但产物不能依赖
构建机 conda 的路径；安装包需在无 conda 的干净 24.04 机器上再做验收。

```bash
python packaging/firmware_updater/build_deb.py --output dist
```

构建工具使用临时目录冻结应用，执行不连接硬件的 `--self-test`，再生成 `.deb`。
安装后可运行 `taccap-firmware-updater --self-test` 检查嵌入式 SDK/Python；
`--ui-smoke-test` 打开窗口后自动关闭，不扫描设备、不刷写。
源代码更改后需要重新打包，不要把运行中的安装包与源码视为同步。
构建需要 `firmware/motor/` 中对应的两份原始电机镜像（EL05 已跟踪，RS00 本地提供）；
缺失或哈希不符时构建失败，不能生成缺省镜像残缺的安装包。
`--self-test` 也会校验随包四份镜像；`--ui-smoke-test` 明确禁用自动连接。

### 等待与同版本处理

- 不再等待 MCU 时钟达到 5 秒；读到基线后立即显示断电提示。重启仍需时钟回退及后续增长，USB 重连不等同断电证明。
- 断电弹窗每约 3 秒显示实际探测结果（在线时钟、找不到设备或通信错误），不再只显示等待动画。
- 同版本确认可选“跳过此项”，不执行该项刷写及刷写后的断电。电机版本仍通过 Private 实时读取，不以 Flash 存档代替；跳过后也必须恢复 MIT。
- 电机刷后仍处于 Private 时，等待自动重启后直接读取实时版本，把必要的刷后硬重启与恢复 MIT 合并，不再先强制重启一次 Private。已经配置好的 EL05 从 MIT 开始：仅刷电机通常为两次 24 V 断电，同时实际刷 MCU 通常为三次（原来分别为三次、四次）。MCU 刷后硬重启仍单独保留，不能在软重启后的通信状态下省略。
- RS00 等电机可能刷后自行回到 MIT；此时为查询实际电机版本仍需重新进入 Private 并断电，可能比上例多一次。不采用主干电机示例中按镜像写入 RS00 版本记录的捷径。型号初始化、CAN ID 修正、初始协议不同或故障恢复也可能增加步骤；不承诺所有设备固定两次断电。
- MCU 版本校验只查询 MCU，不再附带最多 3 秒的电机版本查询。实际提速取决于连接和人工断电时间，不保证固定总时长。
- 界面蓝色表示进行中、绿色表示就绪/完成、橙色提示人工操作、红色提示异常；文字提示始终保留。

### 固件随包更新与日志布局

默认镜像从审核清单中按型号选最高版本；打包检查主/从 MCU 的版本与文件路径必须和
`firmware/manifest.json` 一致，不一致即失败。维护者先审核新镜像来源、兼容性及哈希，
更新清单后再构建安装包。安装后的程序不会自行联网获取新镜像；电机“最新”仅指审核清单
中的最高版本，不声称是厂商网站的最新版本。

主爪模式隐藏电机固件和型号设置，执行前和刷写前分别阻止主/从镜像混刷。
日志展开/收起按可见内容重新计算窗口高度，不再使用固定高度或中间弹性空白。

切换角色筛选后，即使没有设备连接，也立即使用对应角色的默认 MCU 镜像与提示；
主爪隐藏整行电机控件。未连接/未就绪时禁用开始按钮。自动刷新不重复输出未变更
镜像的哈希日志。界面只保留系统标题栏标题，类型、设备、固件使用统一网格对齐。

## 免安装单文件

构建同时输出 `taccap-firmware-updater_0.1.0_amd64.run`，适用 Ubuntu 24.04 x86_64。
它是自解压可执行文件，不需要 conda 或 FUSE；首次运行前按需赋予执行权限：

```bash
chmod +x taccap-firmware-updater_0.1.0_amd64.run
./taccap-firmware-updater_0.1.0_amd64.run
```

启动时临时解压 Python、Qt、SDK 和固件，退出后清理；不要以 root 或 sudo 运行。
仍需系统图形库（同 deb 的依赖）和 dialout 串口权限；便携版不自动注册系统菜单。
下载来源必须可信，文件本身为可执行程序。可用 `--self-test` 检查隔离环境及固件。
标题显示应用版本 `v0.1.0` 与 Git build SHA，不显示打包修订号。
应用、文件名与 Debian 包统一使用 `0.1.0`，不追加 `-数字` 修订号。
后续打包更新递增正式版本号，以保证 apt 升级顺序。
本次重新以 `0.1.0` 为版本基线；已安装更高编号测试包的机器会将其视为降级，
请交互式执行上述 `apt install` 并核对降级提示，不使用自动确认参数。
未提交源码构建时附带 dirty，源码预览标记 dev。
这不是 MCU 或电机版本；实际设备版本仍显示在设备信息中。

### 设备版本颜色

MCU 与电机分别对比安装包内置的最高审核版本：一致为绿色，低于内置版本为红色并提示更新。
未知或型号未记录时不判定为最新；高于内置版本时提示核实，不建议降级。
电机 Flash 存档保留非实时标注，绿色仅表示存档版本一致，不代替实时升级校验。
手动选择回滚镜像不会改变此比较基准；不联网查询厂商版本。

### AppImage 单文件

额外提供 `taccap-firmware-updater_0.1.0_x86_64.AppImage`，与源 deb 使用相同的程序、
内置固件和 build SHA。赋予执行权限后可在 Ubuntu 24.04 x86_64 双击运行；
下载或复制后权限可能丢失，需在文件属性中勾选“允许作为程序执行”。
不需要 conda，不以 root 运行；串口权限仍按上文配置。
系统不允许 FUSE 挂载时可使用 `--appimage-extract-and-run` 启动。

构建使用已审核 SHA256 的官方工具和运行时（不自动信任更新后的 continuous 文件）：

```bash
python packaging/firmware_updater/build_appimage.py \
  dist/taccap-firmware-updater_0.1.0_amd64.deb \
  --tool /path/to/appimagetool-x86_64.AppImage \
  --runtime /path/to/runtime-x86_64
```

工具来源为 AppImage/appimagetool 和 AppImage/type2-runtime 的官方 GitHub Releases。
脚本校验摘要，拒绝覆盖现有 AppImage，也拒绝打包源码未提交的 deb。

### Private 协议未生效时

MCU 时钟重置只证明 MCU 重启，不证明电机供电已经断开。必须按提示断开整只从爪
24 V 至少 2 秒，不能仅拔 USB；程序仍会核验实际协议。

电机刷写前切换 Private 校验失败时，只有重新查询仍为 MIT 才提供一次恢复确认。
用户明确同意后重新请求切换，并重新引导断电、核验 Private；不重复 MCU 刷写。
取消、通信异常、第二次校验失败或电机刷写后的校验失败均停止，不自动重试刷写。
这提供了受控恢复，不表示已确认或修复电机内部参数保存失败的根因。

### 一键打包三种格式

在已配置 PyInstaller、PySide6、patchelf 与匹配原生 SDK 的构建环境执行：

```bash
python packaging/firmware_updater/build_all.py
```

一次冻结生成 deb/run，再从同一 deb 生成 AppImage，三者使用同一 build SHA、
SDK 和内置固件。首次下载官方 AppImage 构建依赖到 `build/appimage-tools` 并验证固定 SHA256，
缓存存在时可离线使用；上游文件摘要变化会拒绝构建，不自动信任新版本。
也可用 `--tools-dir /path/to/verified-tools` 指定缓存。
三种产物全部生成和自检通过后才替换 `dist` 中同名文件，旧文件移至 `dist/previous-build-*`。
构建依赖只用于打包，用户运行仍不依赖 conda。

AppImage 正常启动会为当前用户注册专用菜单入口（不需要 root），并将等比例图标
复制到用户图标主题目录。它使用独立的 `taccap-firmware-updater-appimage` 标识，
不覆盖 deb 的菜单入口；移动 AppImage 后重新直接启动一次可更新入口路径。
`--self-test` 和 `--ui-smoke-test` 不创建用户入口。
可删除 `~/.local/share/applications/taccap-firmware-updater-appimage.desktop` 和对应
`~/.local/share/icons/hicolor/512x512/apps/taccap-firmware-updater-appimage.png` 移除入口
（设置 XDG_DATA_HOME 时使用该目录）。
文件管理器中的 AppImage 文件缩略图由系统支持决定，内嵌图片不保证文件列表自动显示。
