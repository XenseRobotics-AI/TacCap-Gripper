# Prebuilt TC-GU-01 firmware

The current firmware images, so you can upgrade a gripper without access to the
firmware source — which is a separate, internal repository and is **not** part
of this SDK.

| Image | Role | Version | Protocol | Size | CRC32 |
| --- | --- | --- | --- | --- | --- |
| `tc-gu-01-master-1.2.6.bin` | leader (SN ends **`m`**) | **1.2.6** | V2.4 | 118,236 B | `0xbd8e47df` |
| `tc-gu-01-slave-1.2.6.bin` | follower (SN ends **`s`**) | **1.2.6** | V2.4 + 运动安全包络 | 163,000 B | `0xbffbadc8` |

Only the current release is kept here. Older images come from this directory's
git history rather than from extra files.

`manifest.json` carries the same data machine-readably, and it is load-bearing
rather than descriptive: `ota_update.py` resolves which file to send through it,
and checks the image by **CRC32** rather than trusting the filename. Bumping a
version therefore means replacing the `.bin` *and* updating the manifest in the
same change — `python/tests/test_ota_update_all.py` fails if they disagree.

Both roles carry the same version number by policy. Since 1.2.5 the two roles
build from a single version definition, so a change touching only one role still
rebuilds the other; that is the cost of never having two devices report the same
number while running different code.

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
# -> 0xbd8e47df
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

"Power-cycle" here means **unplug the USB cable and the power cable together,
and have both out at the same time**, then plug them back in. The two feed
different domains — USB powers the MCU and its USB-serial bridge, the 24 V line
powers the motor — so pulling either one on its own leaves the other half
energised. Pulling them one after another, with the first already back in, never
gives the board a moment with no power at all, which is the thing that resets
it.

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
