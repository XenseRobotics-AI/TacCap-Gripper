# Prebuilt TC-GU-01 firmware

The **latest released** firmware images, so you can upgrade a gripper without
access to the firmware source repo (which is internal and stays out of this
SDK — see the repo README's "Firmware / PC GUI reference repos").

Only the current release lives here. Older images are recoverable from this
directory's git history, not from extra files.

| Image | Role | Version | Protocol | Source | Size | CRC32 |
| --- | --- | --- | --- | --- | --- | --- |
| `tc-gu-01-master.bin` | leader (SN ends **`m`**) | **1.2.5** | V2.3 | `5e4ea62` | 118,236 B | `0x42c83731` |
| `tc-gu-01-slave.bin` | follower (SN ends **`s`**) | **1.2.5** | V2.3 + envelope | `5e4ea62` | 162,468 B | `0xd0ef491a` |

Both from firmware branch `feat/actuator-can-timeout`, **one commit, one
version number**. `manifest.json` has the same data machine-readably.

1.2.5 is where one-number-per-release became structural: 1.2.3 aligned the two
numbers but left two `#ifdef`-guarded version definitions in the source, so the
cause of the drift survived the fix. 1.2.5 collapses them into a single
definition — a role can no longer be left behind. The per-role change history is
kept as comments next to it, because *what* changed on which side is still worth
tracing; only the number has one source now.

1.2.3 is where the two version lines merge. They had drifted apart (leader
1.2.x, follower 1.1.x) on the theory that V2.2 commands were follower-only and
the leader had no reason to be rebuilt — but the two roles share
`protocol_handler.c`, so a change to the shared layer obliges both. This
release changes exactly that (failed commands now answer with `cmd == 0`), and
with separate lines it was easy to bump the follower and forget the leader:
two leaders ended up reporting 1.2.1 while running a binary 17,809 bytes
different from the official 1.2.1. One number per release makes that
impossible. The cost is that a change touching only one role still bumps the
other.

> ### ⚠️ Both images are local builds
>
> Neither `.bin` here came from the firmware team's release toolchain — both
> were built with `arm-none-eabi-gcc 13.2.1`. Their size and CRC32 are
> therefore **not comparable** with pre-1.1.2 entries: rebuilding the older
> 1.1.1 image from its own commit with this toolchain also fails to reproduce
> it (150,044 B against the shipped 149,256 B), so a few hundred bytes of any
> size delta is toolchain, not firmware code.
>
> **Both are hardware-validated**, on two units each, all four power-cycled at
> 24 V before measuring.
>
> Follower 1.2.5, on `TCGU01A28Z0018s`: the closed zero written by
> auto-calibration (`min_open_rad`) goes to **0**, so normalized 0.0 is the
> closed hard stop itself rather than an inset short of it. Six open/close
> cycles land at raw **−0.00049** with 0.383 mrad of spread, `arrived` true and
> `HOLDING_POSITION` throughout — against −0.02014 on 1.2.3, i.e. 19.65 mrad
> tighter. A five-minute continuous hold does not drift, decays 1.2% in torque,
> and warms **+1.0 °C**, flat within the sensor's 1 °C resolution after 90 s.
>
> The two earlier insets were both measured against premises that no longer
> held. 20 mrad came from the pre-`5b8b3bf` kd-form control law and was never
> revisited when the law changed; 5 mrad assumed a residual the control could
> not close. A pure feed-forward sweep (`kp=kd=0`) settled it: 0.05 N·m → raw
> −0.00249, 0.10 → −0.00096, **0.15 → +0.00000**, and 0.20 / 0.30 / 0.40 / 0.50
> all → +0.00000 unmoved. raw 0 is a hard stop, so no inset is needed and more
> torque buys no closure. Seating it is the SDK's job now
> (`ForcePositionConfig::close_preload_nm`, 0.25 N·m by default).
>
> Leader 1.2.5 is **byte-identical to 1.2.3 except the version byte** (two bytes
> differ). This release touches follower-only code; the leader is rebuilt solely
> to keep the roles on one number, which is the documented cost of that policy.
>
> Follower 1.2.3, on `TCGU01A28Z0018s` and `TCGU01A28Z0015s`: auto-calibration
> completes, force-position control reaches 1.000 / 0.001 / 1.000 with
> `arrived=True`, and instruction-5 fault reads keep advancing. Two things are
> new and were checked directly. The idle status stream stays fresh —
> `status_timestamp_ms` advances ~2000 ms over a 2 s idle window, where it used
> to freeze indefinitely (the firmware asked the motor for active reporting,
> the motor does not support it, and the firmware stopped polling on the
> strength of having asked). And power-on calibration went 11 s → 1.3 s via a
> two-stage approach, with 0.77 mrad of spread over five power cycles against a
> 29 mrad open-limit margin.
>
> Leader 1.2.3, on `TCGU01A28Z0023m` and `TCGU01A28Z0024m`: encoder and IMU
> both read (accel magnitude 9.72 / 9.64 m/s² at rest), the data stream holds
> ~100 Hz over 2 s, and an unassigned command code answers
> `ProtocolError(NACK: InvalidCmd)`. That last one is the whole reason the
> leader had to be reflashed at all: it proves the V2.3 error semantics reached
> the shared protocol layer on this role too. Key handling carries an upstream
> change (`823e351`, timing constants), validated separately.
>
> The earlier leader 1.2.2 measurements still stand, since 1.2.3 contains that
> code — leader 1.2.2, upgraded from 1.2.0: IMU and encoder both stream at ~99 Hz
> under a concurrent 100 Hz command load, with zero retries and zero ACK
> timeouts. The comparison against 1.2.0 on the same bench is the point —
>
> | | IMU / encoder | resync bytes |
> | --- | --- | --- |
> | 1.2.0 | 97.7 / 99.4 Hz | 5681 |
> | 1.2.2 | 100.0 / 100.0 Hz | 0 |
>
> — and under an unthrottled command flood the gap is starker still: 1.2.0's
> stream was starved to **0 Hz** while it processed 34k commands, where 1.2.2
> kept streaming and processed 294k. That is the blocking-log cost, and it was
> always there on the leader too.
>
> **Leader 1.2.2 still replaces an *official* 1.2.1**, so the provenance trade
> stands even though the validation gap is closed: three shared-code defect
> fixes against an artifact the firmware team built and signed off.

**Leader 1.2.2 = 1.2.1 plus the three shared fixes below.** The version bump
exists so the two builds are distinguishable: everything from 1.1.3 onward
changed the leader's binary too, while its version constant stayed at 1.2.1.
It also picks up the two diagnostic commands (`0x54` UART counters, `0x55` log
switch), which are registered in the common command table.

**Leader 1.2.1 changed the status LED only** — protocol byte-identical to
1.2.0:

- normal state: solid **white** at brightness 20 (was solid green at 10)
- fault state: blinks at 500 ms (was 1000 ms)
- the key-press LED reactions (click-blink, long-press-solid) are disabled

**Follower 1.1.2 adds command set V2.2** — the four follower diagnostic
commands (`0x3A` / `0x3B` startup limit torque, `0x52` fault report, `0x53`
extended motor status). It is purely additive: `GetMotorStatus` (0x50) and the
motor-status DATA stream still carry the same 31-byte payload, so everything
the SDK did against 1.1.1 behaves identically. Upgrade only if you want the
diagnostics — on 1.1.1 those four raise `ProtocolError(InvalidCmd)`.

It also retunes power-on auto-calibration: single-sample stall confirmation,
the open stall records the frame *before* the trigger, and 0.013 rad is
subtracted from the saved `max_open` as a safety margin. **Re-run calibration
after upgrading** if you depend on the exact span — expect it slightly smaller.

**Follower 1.1.3 / 1.1.4 / 1.1.5 fix three real defects**, all in code the
leader shares. The protocol surface is unchanged apart from two added
diagnostic commands, so the SDK behaves identically otherwise.

- **1.1.3 — the command channel could be livelocked into permanent silence.**
  Sustained high-rate input made the command task log one blocking UART line
  per received frame; at 1000 frames/s the printing alone needed more than a
  second per second, the RX ring buffer overflowed, and the overflow handler
  logged *from the interrupt*, amplifying it. The device kept streaming at a
  healthy 100 Hz the whole time, which is what made it so hard to spot: every
  command timed out, nothing recovered, and only a power cycle brought it back.
  Also adds `CMD_GET_UART_STATS` (0x54) — the counters the SDK's
  `g.diagnostics.uart_stats()` reads.
- **1.1.4 — logging is off by default**, with `CMD_SET_LOG_CONFIG` (0x55) to
  turn it back on at runtime. Deleting individual log lines only treats the
  symptom: the sink is a blocking polled UART write (~0.5 ms per line at
  921600) that stalls whichever task emitted the line, so *any* chatty code can
  reproduce 1.1.3's failure. The firmware was emitting 34.8 KB/s of it while
  completely idle. Command latency dropped from 877 µs to 489 µs as a
  side-effect. Treat 0x55 as a diagnostic lever, not a setting.
- **1.1.5 — a one-byte out-of-bounds write on every boot.** The UART ring-buffer
  array is sized to exclude the DEBUG port, but 16 of its 17 access sites
  validated the index against the full port range, which includes the one index
  past the end. `log_init()` hit it every startup. What it corrupted depended on
  linker layout.

**Follower 1.1.6 adds the motion safety envelope** — the reason it is a
mandatory upgrade rather than a recommendation, and why `FollowerGripper`
refuses to open anything older.

- **Every MIT frame is now clamped in firmware.** Before 1.1.6 the MIT command
  path had *no* stall protection at all: `can_motor_gripper_stop_on_limit_stall()`
  is wired only into the velocity commands and only near the travel ends, so a
  blocked jaw let `kp * error` grow until the motor's own `0x700B` ceiling.
  Measured at 24 V with `kp=20` and a rigid object: the command asked for
  ~12 Nm, the jaw hit the object at 5.5 rad/s, and the current draw browned out
  the whole board — the gripper dropped its payload and the USB link vanished.
  The envelope clamps commanded position against `peak_torque_nm / kp` and the
  feed-forward term against `cont_torque_nm`, every cycle.
- **I²t derate at zero speed** and a **configurable temperature wall**
  (defaults 90 °C derate start / 100 °C wall). The motor's own over-temperature
  protection did not act at 100 °C case temperature over a 297 s hold, so this
  is the only thermal defence on that axis.
- **`min_open_rad` now stores the control-reachable travel**, symmetric with the
  existing `OPEN_LIMIT_MARGIN_RAD` at the open end.
- The envelope rides inside `gripper_config_t.reserved[16]`, so **commands
  `0x66`/`0x67`, their payload length and the struct size are all unchanged** —
  no protocol change. A device that has never had one written reads back
  `flags = 0` and behaves exactly as before, so **upgrading changes nothing
  until you write an envelope** (`impedance_control.py --set-envelope`).
- Hardware-validated on `TCGU01A28Z0018s` at 24 V: peak speed 8.68 → 1.70 rad/s;
  a stall that should have demanded 5.72 Nm settled at 1.569 Nm; and `kp` raised
  8 → 120 (15×) moved the resulting torque by 1.4%, which is the strongest
  evidence the clamp is real — it depends on no absolute number.
- **`min_open_rad` is only written by the next power-on auto-calibration**, so
  the mandatory post-flash power cycle matters here for a second reason beyond
  the usual bank-swap frame loss.

---

## ⚠️ Power-cycle the gripper after flashing

The bank-swap reboot is a **soft** reset: it restarts the MCU but never powers
the USB-serial bridge down. The device comes back in a degraded state that is
indistinguishable from a healthy one — right version string, stream running,
`uart_stats()` counters all clean — while quietly dropping status frames.

Measured on hardware, same unit, same firmware, same cable, 60-second runs:

| | status frames lost |
| --- | --- |
| after OTA alone | 35–39 per run, three runs |
| after unplug + replug | **0**, three runs |

This cost us two wrong conclusions before we caught it: first a unit was
written off as having a degrading cable, then two firmware versions were
compared using numbers that only differed by whether the gripper had been
power-cycled. **Any measurement taken before the replug is suspect.** Firmware
tracking: tc-gu-01 issue #6.

## ⚠️ Update the SDK *before* flashing

If you are on an SDK older than 0.1.7, install the new SDK **first**, then
flash. Not the other way round.

Older `OtaSession` only checked `ack.is_nack`, but firmware handler errors
come back on the echoed-command path — the command byte intact and the error
as the single payload byte — which the transport cannot tell from a valid
1-byte success. So a rejected write did not raise: the loop wrote every
remaining block, `verify()` and `apply()` swallowed their errors too, and the
update **reported success on a firmware that had aborted the session**. It
also retried with a fresh sequence number, which the firmware sees as a new
request; since it requires strictly sequential offsets and fails the whole
session on a repeat, a merely-slow ACK could kill an otherwise fine update.

Both are fixed in 0.1.7. The new SDK talks to old firmware fine — everything
up to command set V1.9 is unchanged — so upgrading the SDK first is safe.

```bash
pip install -e .          # or however you install this SDK
```

## Flashing

```bash
# 1. Which grippers are attached, and what are they running?
python python/examples/fisheye_cal.py show

# 2. Flash. --side picks the gripper; the image must match the ROLE.
#    Naming the image is enough — the script looks here for it, so this
#    line also works from a parent repo that vendors this one.
python python/examples/ota_update.py tc-gu-01-master.bin \
    --side left --target-version 1.2.1

# 3. Power-cycle the gripper. Not optional — see below.
# 4. Confirm — GetVersion returns the compiled-in constant, so the version
#    you read back is proof of what actually landed.
python python/examples/fisheye_cal.py show --sn <SN>
```

Takes about a second. The MCU reboots and re-enumerates over USB in ~1–3 s.

### Pick the image by ROLE, not by side

A gripper's role is the **last character of its firmware SN**, not which hand
it is on: `TCGU01A28Z0023m` → `m` → **master**. Two grippers on opposite sides
of the same rig are often both masters.

Flashing the wrong role's image bricks the MCU and needs an SWD probe to
recover. Check first:

```bash
python -c "from xense.taccap import scan_grippers
for g in scan_grippers(): print(g.firmware_sn, '->', 'master' if g.firmware_sn.endswith('m') else 'slave')"
```

### Verify before you flash

The manifest's CRC32 is the same value `ota_update.py` prints and sends in
`OtaStart`, so it is worth a look if the file has travelled:

```bash
python -c "
from xense.taccap import crc32_iso_hdlc
print(hex(crc32_iso_hdlc(open('firmware/tc-gu-01-master.bin','rb').read())))"
# → 0xec491cbd
```

## How OTA works, briefly

The image is written to the **inactive** flash bank and activated with the
STM32H5 bank swap, so the plain `.bin` serves both banks — it always links at
`0x08000000` and the swap remaps the target bank there. Nothing is overwritten
until `OtaApply`, and the firmware refuses to swap on a CRC mismatch, so a
failed transfer leaves the running image intact.

No SWD probe is involved; it goes over the same serial link as sensor I/O.

## Rebuilding these

The images are build artifacts of the internal firmware repo. See the main
README's [firmware section](../README.md#firmware--pc-gui-reference-repos)
for the toolchain and the two traps (conda's exported host `CFLAGS` breaking
the ARM cross-compile, and the 456 KB single-bank OTA cap).

When you refresh these files, regenerate `manifest.json` too — the CRC32 in
it is what people check against.
