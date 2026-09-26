# Prebuilt TC-GU-01 firmware

The current firmware images, so you can upgrade a gripper without access to the
firmware source — which is a separate, internal repository and is **not** part
of this SDK.

| Image | Role | Version | Protocol | Size | CRC32 |
| --- | --- | --- | --- | --- | --- |
| `tc-gu-01-master-1.2.5.bin` | leader (SN ends **`m`**) | **1.2.5** | V2.6 | 118,220 B | `0x885b8706` |
| `tc-gu-01-slave-1.2.9.bin` | follower (SN ends **`s`**) | **1.2.9** | V2.6 + 运动安全包络 | 165,056 B | `0xc69f7015` |

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

The leader is at 1.2.5 while the follower is at 1.2.9 for that reason: 1.2.7 and
1.2.8 touched only follower code, and 1.2.9 changed the shared `storage.c`, which
is what moved the leader from 1.2.4 to 1.2.5 — with no change in its behaviour.

## Follower: record the motor model once

The follower image is built with the **EL05** as its compile-time *fallback*
motor. The model that actually counts is recorded per device in MCU flash,
because the motor cannot report it and every MIT frame is quantised against the
model's torque and velocity ranges — the wrong model is not an error, it is a
constant factor on every frame (EL05 ±6 N·m vs RS00 ±14 N·m).

A follower built around an **RS00** therefore needs its model written once, then
a power cycle:

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
print(hex(crc32_iso_hdlc(open('firmware/tc-gu-01-master-1.2.5.bin','rb').read())))"
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
three 24 V-only cycles each restarted the MCU, and after an OTA plus one such
cycle a 60 s status stream lost 0 of 6000 frames. A **leader** has no 24 V
rail: unplug and replug its USB. To confirm the cycle really happened,
`gripper.device.heartbeat().uptime_ms` restarts near 0. Do not go by the `/dev/serial/by-id/` timestamp: a
24 V-only cycle restarts the MCU without always re-enumerating USB.

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
