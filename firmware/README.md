# Prebuilt TC-GU-01 firmware

The current firmware images, so you can upgrade a gripper without access to the
firmware source — which is a separate, internal repository and is **not** part
of this SDK.

| Image | Role | Version | Protocol | Size | CRC32 |
| --- | --- | --- | --- | --- | --- |
| `tc-gu-01-master-1.2.6.bin` | leader (SN ends **`m`**) | **1.2.6** | V2.6 | 118,300 B | `0x350393b5` |
| `tc-gu-01-slave-1.2.14.bin` | follower (SN ends **`s`**) | **1.2.14** | V2.8 + 运动安全包络 | 167,168 B | `0x267d7f67` |

Only the current release is kept here. Older images come from this directory's
git history rather than from extra files.

`manifest.json` carries the same data machine-readably, and it is load-bearing
rather than descriptive: `ota_update.py` resolves which file to send through it,
and checks the image by **CRC32** rather than trusting the filename. Bumping a
version therefore means replacing the `.bin` *and* updating the manifest in the
same change — `python/tests/test_ota_update_all.py` fails if they disagree.

**The two roles carry independent version numbers**, so the table above will
usually show two different values. Each role has its own definition in the
firmware's `protocol_handler.c`.

They were briefly forced onto one number: 1.2.3 aligned the digits and 1.2.5
collapsed the two definitions into one, because a shared-layer change had once
been bumped on the follower and forgotten on the leader — two leaders then ran
binaries 17,809 bytes apart while both reporting 1.2.1. That guard was dropped on
2026-09-24 in favour of a rule the firmware repo states explicitly: **a change to
shared code bumps both roles**, and only a change confined to one role's own
sources bumps that role alone.

**A leader reporting 1.2.6 may be the old aligned-number image** (same code as
1.2.4, no receive fix) rather than the shipped one. The number collides; reflash
this directory's `tc-gu-01-master-1.2.6.bin` if you are not sure. See
docs/FIRMWARE.md.

The leader is at 1.2.6 while the follower is at 1.2.14 for that reason: 1.2.7,
1.2.8, 1.2.10, 1.2.12, 1.2.13 and 1.2.14 touched only follower code, and 1.2.9 changed the shared
`storage.c`, which is what moved the leader from 1.2.4 to 1.2.5 — with no change
in its behaviour.

**Follower 1.2.11 fixes a command channel that could go silent for good.** The
control UART (3 Mbaud, one interrupt per byte) stopped receiving after a single
overrun, because the error callback never restarted reception: the status
stream kept flowing, every command went unanswered, and 300 ms later the
host-timeout safe hold dropped the grip to 0.35 N·m. Measured on 0086s after
about 3 minutes of a 3 N·m ForcePosition hold; gone on 1.2.11. Reception is now
restarted, `g.diagnostics.uart_stats()` reports `rx_errors` / `rx_rearms`
(`rx_rearms > 0` means it happened and was recovered), and a host-timeout
disable keeps `stop_reason == HostTimeout`. The receive path is shared, so the
leader got the same fix as **1.2.6**, verified on 0115m and 0116m: 15 minutes
each of a 100 Hz stream plus a 100 Hz acknowledged command, ~87,000 commands, no
failures.

**Follower 1.2.12 makes the model-dependent settings follow the model.** Before
it, a follower upgraded from 1.1.x kept EL05-era values after the model was
identified as RS00: 0x700B at 6.0, the auto-calibration stall torque at 0.35 N·m
(too little to move an RS00, so calibration stopped short of the stops and could
record the open direction backwards -- 0094s did), the open direction defaulting
to "reverse" (right for EL05, wrong for RS00), and no motion envelope at all
unless someone ran `ensure_envelope()`. Now:

| | EL05 | RS00 |
|---|---|---|
| auto-cal stall torque default | 0.5 N·m | 1.8 N·m |
| default open direction | reverse | not reverse |
| envelope when none stored | cont 1.1 / peak 1.8 | cont 3.6 / peak 5.0 |

Peak is the rated torque since 1.2.13 (1.2.12 used `t_max`, which left the
position-error clamp with nothing to do). The envelope default has the 90/100 °C temperature wall, measured on EL05; RS00
is not separately characterised. It is computed at run time, never written, so a
stored envelope still wins; `get_model().default_envelope == 1` says the default
is in force. When the model record changes, 0x700B, the calibration torque and
the direction are reset to the new model's defaults; a stored calibration torque
still at exactly the old 0.35 migrates on boot. A gripper OTA also no longer
refuses when the motor is not on the bus.

**Follower 1.2.14 records the motor's firmware version in flash** (0x5C to
write it, 0x58 falls back to it under MIT) -- see the SDK README, *RobStride
motor firmware*. Verified on 0094s: written, read back under MIT, kept across a
24 V power cycle.

Not verified on hardware: OTA with the motor off the bus, and calibration from an
invalid config using the model's default direction.

## Follower: the motor model

Every MIT frame is quantised against the motor model's torque and velocity
ranges, so the wrong model is not an error — it is a constant factor on every
frame (EL05 ±6 N·m vs RS00 ±14 N·m). The model is recorded per device in MCU
flash; the image's compile-time *fallback* is the **EL05**.

**Since 1.2.10 the record fills itself in.** When the motor is on the private
protocol at boot — which a new motor is, since RobStride ships them that way — the
firmware reads the motor's firmware version and maps its raw bytes to a model
(`{10,5,…}` = EL05, which RobStride writes `1.0.5.x`; `{0,0,3,…}` = RS00),
records it, and switches the motor to MIT. So a new or replaced motor needs
nothing. `g.motor.get_model().autodetected == 1` on the boot that did it.

A unit that was already on MIT before 1.2.10 never passes that point. If
`get_model()` reports the compile-time default on one (the SDK warns when it opens
such a follower), write it once and cut 24 V for ~2 s:

```python
g.motor.set_model(1)          # 0 = EL05, 1 = RS00
g.motor.get_model()           # -> MotorModel(RS00, ..., from flash, t_max=14.0Nm, ...)
```

A device upgraded from 1.2.8 or earlier keeps the 0x700B startup torque limit it
had stored (6.0 N·m on every unit so far). 1.2.9 defaults a device that never
stored one to its model's range, but will not overwrite a stored value — on an
RS00 raise it once with `g.motor.set_startup_limit_torque(14.0)` and power-cycle.

## Flashing

```bash
# 1. What is attached, and what is it running?
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.side, g.role, g.firmware_sn)"

# 2. Flash, by role. The script finds the image in this directory.
python python/examples/ota_update.py slave left

# 3. POWER-CYCLE the gripper. Not optional — see below.

# 4. Confirm. GetVersion returns the compiled-in constant, so what you read
#    back is proof of what actually landed.
python python/examples/follower_status.py left
```

The transfer takes about a second; the MCU reboots and re-enumerates over USB in
roughly 1–3 s.

### Pick the image by ROLE, not by side

A gripper's role is the **last character of its firmware SN**, not which hand it
is on — `…0023m` → `m` → master. Two grippers on opposite sides of the same rig
are often both masters.

Flashing the wrong role's image bricks the MCU and needs an SWD probe to
recover, so check first:

```bash
python -c "from xense.taccap import scan_grippers
for g in scan_grippers():
    print(g.firmware_sn, '->', 'master' if g.firmware_sn.endswith('m') else 'slave')"
```

### Verify an image before you flash it

The manifest's CRC32 is the same value `ota_update.py` prints and sends in
`OtaStart`, so it is worth checking if the file has travelled:

```bash
python -c "
from xense.taccap import crc32_iso_hdlc
print(hex(crc32_iso_hdlc(open('firmware/tc-gu-01-master-1.2.6.bin','rb').read())))"
# -> 0x885b8706
```

## ⚠️ Power-cycle the gripper after flashing

The bank-swap reboot is a **soft** reset: it restarts the MCU but never powers
the USB-serial bridge down. The device comes back in a degraded state that is
indistinguishable from a healthy one — right version string, stream running,
`uart_stats()` counters all clean — while quietly dropping status frames.

Measured on the same unit, same firmware, same cable, 60-second runs:

| | status frames lost |
| --- | --- |
| after OTA alone | 35–39 per run, three runs |
| after unplug + replug | **0**, three runs |

**Any measurement taken before the power cycle is suspect.**

"Power-cycle" on a **follower** means **unplug the 24 V power cable, wait
~2 s, and plug it back in**; the USB cable can stay connected. The follower's
MCU and motor run on 24 V, so cutting it restarts both — while pulling only USB
leaves the MCU running on 24 V and resets nothing. Measured on an EL05 follower:
cutting 24 V drops the whole on-board USB hub (serial bridge included) within
0.3 s and restarts the MCU — 15 of 15 cycles, from the quickest tap to 10 s off —
and after an OTA plus one such cycle a 60 s status stream lost 0 of 6000 frames. A **leader** has no 24 V
rail: unplug and replug its USB. To confirm the cycle really happened,
`gripper.device.heartbeat().uptime_ms` restarts near 0. The `/dev/serial/by-id/`
timestamp usually moves too, but uptime is the direct evidence that the MCU restarted.

## How OTA works, briefly

The image is written to the **inactive** flash bank and activated with the
STM32H5 bank swap, so a plain `.bin` serves both banks — it always links at
`0x08000000`, and the swap remaps the target bank there. Nothing is overwritten
until `OtaApply`, and the firmware refuses to swap on a CRC mismatch, so a failed
transfer leaves the running image intact.

No SWD probe is involved; it goes over the same serial link as sensor I/O.

## Rebuilding these

These are build artifacts of the firmware source repository. See
[docs/FIRMWARE.md](../docs/FIRMWARE.md) for the toolchain and the two traps
(conda's exported host `CFLAGS` breaking the ARM cross-compile, and the 456 KB
single-bank OTA cap).

When you refresh these files, regenerate `manifest.json` in the same change —
its CRC32 is what `ota_update.py` verifies against.
