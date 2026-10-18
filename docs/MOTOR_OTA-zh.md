# 升级从夹爪的电机固件

> 本文是 [MOTOR_OTA.md](MOTOR_OTA.md) 的中文版，以英文版为准。

从夹爪里的 RobStride 电机运行着自己的固件，和夹爪主控（MCU）的固件是两套。本文介绍如何用
`python/examples/motor_ota_update.py`，通过夹爪的 USB-C 线升级电机固件。电机不需要拆下来。

| 电机 | 电机固件要求 |
| --- | --- |
| EL05 | ≥ 1.0.5.0.4 |

## 开始之前

- **夹爪固件 1.2.16**（升级电机至少需要 1.2.8）。用 `follower_status.py` 查看，看 `[fw]`
  那一行。如果版本较旧，先用 `ota_update.py` 升级夹爪固件并断电重启。升级夹爪固件会把电机切回
  常规协议，所以必须先升级夹爪、再升级电机，顺序不能反。
- **SDK 0.4.3 或更新版本**，并且 `import xense.taccap` 能正常导入。
- **我们提供的电机固件镜像**，例如 `EL05_1.0.5.0.4.bin`。请保持原文件名：脚本会从文件名读出
  电机型号和版本，并与镜像内容和夹爪记录核对。
- **没有别的程序在使用夹爪。** 先停掉你自己的程序。
- **两指之间清空。** 每次断电重启后，夹爪会自动开合一次做标定。

下文的"断电重启"一律指：拔下夹爪的 **24V 电源线**，等约 2 秒再插回。USB 线可以不拔，夹爪主控和
电机都由 24V 供电。

## 为什么同一条命令要运行两遍

我们的 SDK 让电机工作在 **MIT 协议**下。电机只在**私有协议**下接受固件升级，而切换协议要到下次
上电才生效。所以一次升级的过程是：

1. 运行脚本：脚本把电机切到私有协议；
2. 断电重启；
3. 再运行一遍同样的命令：脚本刷写固件、核对版本、把新版本记进夹爪，如果运动安全包络和电机额定值
   不一致就顺带改正，最后把电机切回 MIT；
4. 断电重启。

私有协议只在升级时使用。全部完成后夹爪回到 MIT，可以正常使用。

## 操作步骤

下面以左侧从夹爪为例；右侧用 `right`，或者用完整序列号（例如 `TCGU01A25Z0017s`）指定。

### 1. 查看当前状态

```bash
python python/examples/follower_status.py left
```

看 `[fw]`（夹爪固件）和 `[motor fw]`（电机固件）两行。此时电机固件可能显示读不到：电机在 MIT 下、
且夹爪里还没有记录时就是这样，属于正常现象。

### 2. 第一遍：切到私有协议

```bash
python python/examples/motor_ota_update.py EL05_1.0.5.0.4.bin left
```

```text
=== motor OTA, step 1 of 2 ===
  the motor is on the MIT protocol and does not answer OTA frames there,
  so it has to be switched to its private protocol first.
Switch the motor to the private protocol? type 'yes': yes

done. Now unplug the gripper's 24 V power cable for ~2 s and plug it back (USB may stay in),
wait ~10 s, then run the same command again to flash.
```

**断电重启**，等约 10 秒。私有协议下电机保持不使能，夹爪不会做标定，这是正常的。

### 3. 第二遍：刷写

```bash
python python/examples/motor_ota_update.py EL05_1.0.5.0.4.bin left
```

```text
=== motor OTA, step 2 of 2 ===
  gripper      : TCGU01A25Z0017s  firmware FirmwareVersion(1.2.16.0)
  motor        : EL05  can_id 1  uid ...
  motor fw now : 1.0.5.0.2
  image        : EL05_1.0.5.0.4.bin
                 92,584 B, 11,573 packs, version 1.0.5.0.4
  envelope     : at the motor's rating, unchanged
Flash the MOTOR with this image? type 'yes': yes
  [########################################] 100.0%  11,573/11,573
=== done in 28s; waiting for the motor ===
  motor fw     : 1.0.5.0.2 -> 1.0.5.0.4
  recorded     : motor fw 1.0.5.0.4 in follower flash
  switched the motor back to the MIT protocol.

Done. Now unplug the gripper's 24 V power cable for ~2 s and plug it back (USB may stay in);
the gripper auto-calibrates (~10 s, keep the jaws clear) and is ready to use.
```

刷写大约需要半分钟。开始时出现 `motor OTA start: no reply (attempt 1/6)` 这类警告是正常的：
电机要先重启进 bootloader，之后的某次重试才会应答。

**运动安全包络。** 如果夹爪里存的运动安全包络（力矩和过热保护）和电机额定值不一致，脚本会在刷写
之后把它改成额定值（EL05：cont 1.1 / peak 1.8 N·m），并在你确认刷写前提示：

```text
  envelope     : stored cont 1.000 / peak 2.000 N*m, will be set to cont 1.100 / peak 1.800 after flashing
```

行程标定和开合方向保持不变。加 `--keep-envelope` 可以不改包络。

### 4. 断电重启并确认

**断电重启**，等夹爪标定完（约 10 秒），然后运行：

```bash
python python/examples/follower_status.py left
```

`[motor fw]` 应显示新版本，例如 `MotorVersion(1.0.5.0.4, flash: host-written)`。版本已经记在
夹爪里，之后在 MIT 下也能直接读到。

## 出现问题时

| 现象 | 处理方法 |
| --- | --- |
| `cannot tell the motor model from ...` | 镜像文件被改了名。恢复原文件名，或加 `--model EL05`。 |
| `this gripper records a ... motor, the image is for ...` | 镜像对应的是另一种电机型号。不要强行刷写，请联系我们。 |
| `follower firmware ... lacks the 0x5B relay` | 先升级夹爪固件（见"开始之前"）。 |
| `motor OTA ...: no reply after N attempts` | 断电重启后再运行一遍同样的命令。中断的升级会从头开始，电机不会损坏。 |
| 刷写后提示 `!! expected ...` | 断电重启后再运行一遍同样的命令。 |
| 脚本中途停止，之后夹爪不再标定 | 电机还停在私有协议。运行 `python python/examples/motor_ota_update.py --to-mit left`，再断电重启。 |

不刷写、只把电机切回 MIT：

```bash
python python/examples/motor_ota_update.py --to-mit left
```

然后断电重启。

## 在自己的代码里调用

这个脚本只是对 `xense.taccap.MotorOtaSession` 的简单封装：

```python
import xense.taccap as t

g = t.FollowerGripper(t.find_left().mcu_device)
m = g.motor
# the motor must already be on the private protocol (see step 2)
session = t.MotorOtaSession(m, m.get_can_id())
session.preflight("EL05")                       # private protocol + recorded model
session.update_from_file("EL05_1.0.5.0.4.bin", lambda done, total: None)
m.set_motor_fw_version("1.0.5.0.4")             # record it for MIT reads
m.switch_protocol(t.MotorProtocol.Mit)          # then power-cycle
```
