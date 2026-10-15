# Prebuilt TC-GU-01 firmware

Current firmware images for the TacCap-Gripper MCU. The firmware source is not
part of this SDK.

| Image | Role | Version | Protocol | Size | CRC32 |
| --- | --- | --- | --- | --- | --- |
| `tc-gu-01-master-1.2.6.bin` | leader (SN ends **`m`**) | **1.2.6** | V2.6 | 118,300 B | `0x350393b5` |
| `tc-gu-01-slave-1.2.16.bin` | follower (SN ends **`s`**) | **1.2.16** | V2.8 | 167,544 B | `0x4a8cd360` |

Only the current release is kept here; older images are in git history.
[`manifest.json`](manifest.json) carries the same data machine-readably —
`ota_update.py` picks the file through it and verifies the image by CRC32.

Leader and follower firmware are **versioned independently**; different numbers
on the two halves of a pair are normal. The SDK refuses to drive a follower
below **1.2.5**.

## Flashing

```bash
# 1. What is attached, and which role is it?
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side, g.role, g.firmware_sn)"

# 2. Flash by role. The script finds the image in this directory.
python python/examples/ota_update.py slave left      # or: master right / --all

# 3. Power-cycle (see below).

# 4. Confirm the version that actually landed.
python python/examples/follower_status.py left
```

The transfer takes about a second; the MCU reboots and re-enumerates in 1–3 s.
The image is written to the inactive flash bank and activated only after a
CRC check, so a failed transfer leaves the running image intact. No SWD probe
is needed.

> [!WARNING]
> **Pick the image by role, not by side.** The role is the last character of
> the firmware SN (`m` = master/leader, `s` = slave/follower). Flashing the
> wrong role's image bricks the MCU and needs an SWD probe to recover.

To verify an image file before flashing:

```bash
python -c "from xense.taccap import crc32_iso_hdlc
print(hex(crc32_iso_hdlc(open('firmware/tc-gu-01-master-1.2.6.bin','rb').read())))"
# -> 0x350393b5
```

### Power-cycle after every flash

The post-flash reboot is a soft reset that does not restart the USB-serial
bridge; the device then looks healthy while quietly dropping status frames.

- **Follower:** unplug the **24 V** power cable, wait ~2 s, plug it back. USB
  can stay connected — unplugging only USB does *not* reset a follower.
- **Leader:** unplug and replug its USB cable.

`gripper.device.heartbeat().uptime_ms` restarting near 0 confirms it worked.

## Follower motor setup

A follower carries a RobStride motor, either an **EL05** or an **RS00**. These
settings are stored in the follower's flash and normally done once at the
factory; after changing any of them, cut 24 V for ~2 s.

**Motor model.** Since firmware 1.2.10 the follower detects the model itself the
first time a new motor boots. Check it with `g.motor.get_model()`. If the SDK
warns that the model is the compile-time default, set it once:

```python
g.motor.set_model(1)                         # 0 = EL05, 1 = RS00
g.motor.set_startup_limit_torque(14.0)       # RS00 upgraded from firmware <= 1.2.8 only
```

**RS00 handedness.** The RS00 gear train is mirror-mounted, so left and right
followers open in opposite directions: left (odd SN) stores flags `0x0003`
(Reverse 1), right (even SN) `0x0001` (Reverse 0). Firmware 1.2.15+ sets this
from the SN during auto-calibration. `FollowerGripper` warns on open when the
stored direction disagrees with the side; `t.expected_open_reverse(model_id,
side)` gives the correct value, and `g.set_gripper_config()` writes it.

**RS00 CAN ID.** RS00 followers need motor CAN ID **17 (left) / 18 (right)**,
not the factory default 127 — with 127, force-position control cannot close a
left-hand gripper fully. Set it with `g.motor.set_can_id()`;
`t.expected_motor_can_id(model_id, side)` gives the correct value, and
`FollowerGripper` warns on open when it disagrees. EL05 is not affected.

### Motor firmware

The motor runs its own firmware, separate from the gripper MCU.

| Motor | Required firmware |
| --- | --- |
| EL05 | ≥ 1.0.5.0.4 |
| RS00 | ≥ 0.0.3.32 |

Older motor firmware produces wrong velocity feedback (EL05) or motion jitter
(RS00). Read the version with `g.motor.motor_version()`. The motor answers it
only under its private protocol; with follower 1.2.14+ a version stored in flash
is returned under MIT as well — record it after each motor update with
`g.motor.set_motor_fw_version("0.0.3.32")`.

**Updating the motor over the gripper's USB-C** (follower firmware ≥ 1.2.8; the
motor stays mounted):

```bash
python python/examples/motor_ota_update.py rs00-0.0.3.32.bin left
```

1. Flash the gripper firmware first — a gripper OTA switches the motor back to
   MIT.
2. Switch the motor to the private protocol:
   `g.motor.switch_protocol(t.MotorProtocol.Private)`, then cut 24 V for ~2 s.
3. Run `motor_ota_update.py`. It refuses an image whose model does not match
   the model recorded on the gripper (RobStride's own OTA does not check).
4. Cut 24 V again. The motor comes back on MIT.

## Building from source

See [docs/FIRMWARE.md](../docs/FIRMWARE.md) (requires access to the firmware
repository).
