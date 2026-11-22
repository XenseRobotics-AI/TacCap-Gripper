# 标定

## Calibration

Mechanical slop and small post-zero drift can make the encoder report
~0.05–0.10 rad when the gripper is held "fully closed". The SDK
absorbs this two ways:

- **Auto-clamp**: `Encoder::read_once()` and `on_data` callbacks return
  `position_rad >= 0`. Negative raw drift becomes `0.0` to keep
  downstream consumers' math sane. The unclamped value is preserved
  in `raw.position_rad` (C++) / `raw_position_rad` (Python) for
  diagnostics.
- **Drift warning**: if the raw negative drift exceeds **-0.1 rad** the
  logger emits a rate-limited warning (1 / s per `Encoder` instance)
  pointing at calibration or mechanical issues.

To calibrate a gripper, run `calibrate.py` against the side you want to fix
(or its SN, if you'd rather be explicit):

```bash
python python/examples/calibrate.py left               # by side
python python/examples/calibrate.py TCGU01A24Z0001m    # by firmware SN
```

The script:

1. Resolves `left`/`right` (or the SN you passed) to one `mcu_device`, and
   prints the firmware SN it picked plus every gripper it can see, so the
   pick is verifiable. Side comes from the firmware-burned SN read over the
   wire (`Cmd::GetSn`), not the CH343 chip serial.
2. **Pre-flight:** checks the firmware implements `Cmd::EncoderMaxCal`
   (0x2C). This runs *before* anything is written — step 4 persists a new
   zero, so a pre-V2.1 gripper is refused while still untouched rather than
   left half-calibrated with a new zero and no span.
3. Prints the current encoder reading (both `raw` and clamped) so the
   existing drift is visible.
4. Prompts "hold the gripper **FULLY CLOSED**, press [Enter]", sends
   `Cmd::SetEncoderZero`, re-reads, and validates the new raw reading is
   within ± 0.01 rad.
5. Prompts "open to the **MECHANICAL LIMIT**, press [Enter]" and **stores**
   the measured angle as the travel span (`Cmd::EncoderMaxCal`). That span
   is what `normalize_position=True` divides by.
6. Live 10 Hz readout (`raw | cooked | position 0..1`) until Ctrl+C.

There is deliberately **no expected full-open angle** to check against. The
measured span *is* the calibration — it is whatever the mechanism does, and
it is what the SDK normalizes by. (An earlier 1.7 rad "design baseline" was
stale and fired false alarms on healthy hardware: three measurements across
two units gave 1.1582 / 1.1589 / 1.1486 rad, i.e. ~66°.) The only checks
left are the ones that catch a genuinely broken measurement — a non-positive
span, a zero that did not take, a write that did not stick.

`--skip-open-probe` latches only the zero; normalized position then stays
unavailable until the span is measured.

The firmware latches whatever raw count it sees the moment it
processes the command, so the gripper must already be at the target
pose before pressing Enter.

### Flash-persisted calibration records (V2.0 / V2.1)

Two more records live in MCU flash behind `g.calibration`. Note the encoder
zero above is **also** flash-persisted — `cmd_handler_set_encoder_zero` calls
`storage_write_encoder_calibration()`, so none of these need a power cycle
and none of them are lost on reboot:

| Record | Command | Scope | Accessor |
| --- | --- | --- | --- |
| Fisheye camera `fx, fy, cx, cy, k1..k4` (+ calibration resolution, V2.10; one per resolution, V2.11) | `0x2B` | leader + follower | `read_fisheye_slot()` / `read_fisheye_record()` / `write_fisheye_sized()` / `clear_fisheye_slot()` (legacy: `read_fisheye()` / `write_fisheye()`) |
| Encoder max travel angle (rad) | `0x2C` | **leader only** | `read_encoder_max_rad()` / `write_encoder_max_rad()` |

Both survive power cycles. A record that was never written normally reads back
as `None` — the firmware answers `ErrorCode.CalNotSet` instead of returning
zeros, so "never calibrated" is distinguishable from "calibrated to exactly
0". Every other firmware error still raises `ProtocolError`; on a follower or
on pre-V2.1 firmware the encoder-max methods raise with `InvalidCmd`.

**`None` is not the only way "never calibrated" arrives, though.** An
uncalibrated unit may answer the fisheye read with a record that is *present*
and entirely zero — observed on firmware 1.1.1 and still on **1.2.2**, so treat
it as how the firmware behaves rather than as one old version's quirk. It passes any `if params is None` check, and building remap tables from
`fx = fy = 0` maps every pixel outside the source image — so the "rectified"
frame comes out uniformly black, with nothing raising. Test the record with
`is_usable_fisheye_cal()` (finite, positive `fx`/`fy`) rather than for `None`,
or let `resolve_fisheye()` below do it for you.

```bash
python python/examples/fisheye_cal.py show                  # print both records
python python/examples/fisheye_cal.py measure-encoder-max   # guided: zero, open, store
```

### An uncalibrated wrist lens falls back, it does not degrade to raw

`Calibration::resolve_fisheye()` is the one home for the read-and-fall-back
policy, so every consumer makes the same decision instead of re-deriving it.
**Prefer it over `read_fisheye()`** unless you specifically need to know what
the flash holds.

Both languages return a `ResolvedFisheyeCal` (`calibration`, `is_reference`,
`reason`, `calibrated_size`, `size_inferred`). In Python it still unpacks as the
`(calibration, is_reference, reason)` tuple it used to be, and
`calibrated_size` is a `(width, height)` tuple:

```python
resolved = g.calibration.resolve_fisheye()
cal, is_reference, reason = resolved
if is_reference:
    log.warning("rectifying with the SDK's reference intrinsics: %s", reason)
w, h = resolved.calibrated_size      # open the camera at exactly this size
```

`reason` is non-empty exactly when `is_reference` is true.

`matches_reference` is the opposite case: the unit's own record was used, but it
**is** the reference values (`matches_fisheye_fallback_cal()`), i.e. the factory
stored `FISHEYE_FALLBACK_CAL` instead of calibrating the unit (seen on leader
TCGU01A28Z0096m, 2026-11-20). A warning is logged; recalibrate such a unit.

Three ways a unit fails to supply its own calibration, all of which fall back:

| What happened | How it is detected |
| --- | --- |
| The lens was never calibrated | `read_fisheye()` → `None` (`CalNotSet`) |
| Firmware answered with an all-zero record | `!is_usable_fisheye_cal()` — see above |
| Firmware predates command set V2.0 | `ProtocolError` (`InvalidCmd`) on `0x2B` |
| The record has no resolution and none can be inferred | `infer_fisheye_cal_size()` → `None` — see below |

What stands in is `FISHEYE_FALLBACK_CAL`, a reference calibration measured on a
sample TC-GU-01 and compiled into the SDK
(`cpp/include/taccap/components/fisheye_undistorter.hpp`):

```text
fx 213.0303  fy 212.7928   cx 321.4000  cy 239.9500
k1  -0.0172  k2   0.0091   k3  -0.0146  k4   0.0051
```

They are a **640x480** calibration. Every unit carries the same lens and
sensor, so these shared numbers are much closer to correct than no
rectification at all — which is why
`install_wrist_undistorter()` now installs undistortion **always**, and logs
which of the two calibrations it used.

At **1920x1080** (a different sensor crop, so the 640x480 numbers cannot be
rescaled to it) the reference is `FISHEYE_FALLBACK_CAL_HD`: the element-wise
mean of followers `TCGU01A45A0016s` (31 views, RMS 0.139 px) and
`TCGU01A45A0017s` (22 views, RMS 0.141 px), both calibrated at 1920x1080 on
2026-11-22 (firmware 1.3.1). It replaced the first HD reference (SDK 0.4.9),
leader `TCGU01A28Z0116m`, whose lens sits off-centre (cy ≈ 501 against ≈ 550
elsewhere):

```text
fx 475.6250  fy 474.8844   cx 949.7232  cy 552.1160
k1 -0.016049 k2 -0.004148  k3  0.000200 k4  0.000574
```

`resolve_fisheye_for(1920, 1080)` falls back to it when a unit has no 1080p
calibration, and `fisheye_fallback_cal_for(w, h)` returns the reference for a
size. Both references are stand-ins: every unit should be calibrated at the
resolution it captures at. `matches_fisheye_fallback_cal(cal, w, h)` compares
against the reference for that size. Being an average, the HD reference is no
single unit's calibration (`FISHEYE_FALLBACK_CAL_HD_SOURCE` is the marker
`avg:TCGU01A45A0016s+0017s`, not an SN).

> **Watch out: the optical axis is not necessarily at the image centre.**
>
> A rectified frame can look off-centre, or subtly tilted, **while the
> calibration is perfectly correct**. The sensor is not always mounted exactly at
> the lens's optical centre, and rectification is built around the principal
> point, not around the middle of the frame.
>
> Measured on one TC-GU-01 leader: `cx = 359.1` on a 640-wide frame — the optical
> axis sits **39 px right of the image centre**. Confirmed independently of the
> calibration, from the picture itself: the two gripper jaws are mechanically
> symmetric about the optical axis, and the midpoint between their tips lands at
> x = 360.1, within about a pixel of the calibrated `cx`. Forcing `cx` back to 320
> moved the jaws *further* off centre and introduced a tilt — that is the check
> worth repeating before concluding a calibration is wrong.
>
> Raw fisheye hides this: barrel distortion compresses the periphery, so a 39 px
> offset is not visible. Rectification expands the periphery and the same offset
> becomes obvious. **Rectification reveals the offset, it does not cause it, and
> `cx` is not the thing to "fix"** — an offset this size is an assembly matter.
> If the framing matters, crop around the optical axis afterwards, and update the
> camera matrix to match or anything measuring in pixels will be wrong.
>
> On that same unit `FISHEYE_FALLBACK_CAL` sits 37.7 px away from the real
> optical axis, which is what "approximate" means here in practice.

**It is not a substitute for calibrating a unit.** Lens placement varies between
assemblies, so the principal point in particular drifts per unit. Anything that
measures in pixels off a rectified frame needs this unit's own calibration —
store one with `fisheye_cal.py set-fisheye`. Every fallback path warns and says
so.

### The calibration resolution (V2.10: firmware 1.3.1, or 1.3.0-rc1+)

The 8 floats hold only at the capture resolution they were measured at. The
wrist camera's default moved from 640x480 to 1920x1080, and those are
**different sensor crops** (4:3 vs 16:9), not one image at two scales — so
intrinsics cannot be rescaled from one to the other, and the SDK never tries.

Firmware V2.10 records the resolution next to the params (`0x2B` ops
`0x02`/`0x03`):

| Call | What it does |
| --- | --- |
| `supports_fisheye_size()` | Probes with the sized read: older firmware rejects it as `InvalidParam`. Cached per `Calibration`. |
| `read_fisheye_record()` | `FisheyeCalRecord` (`calibration`, `width`, `height`, `size_known()`); `0/0` = unknown. Older firmware: the legacy read, size `0/0`. `None` when never calibrated. |
| `write_fisheye_sized(cal, w, h)` | Checks `fisheye_cal_size_plausible()` first (the firmware's own rule: finite, `fx, fy > 0`, `1 <= w, h <= 4096`, principal point inside the frame). Older firmware: only 640x480 is accepted, through the legacy write; anything else raises and asks for a firmware upgrade. |

`write_fisheye()` still works, but on V2.10 firmware it marks the size
**unknown**. Store calibrations with `write_fisheye_sized()`:

```bash
python python/examples/fisheye_cal.py set-fisheye --from-npz cam.npz --width 1920 --height 1080
```

A record without a size (legacy write, or written by older firmware) is not
assumed to be 640x480. `infer_fisheye_cal_size()` applies a rule shared verbatim
with the PC calibration tools: the candidates are exactly 640x480 and
1920x1080, a candidate `W x H` matches when the calibration is usable and
`|cx - W/2| <= 0.10*W` and `|cy - H/2| <= 0.10*H`, and a size is returned only
when exactly one candidate matches. `resolve_fisheye()` uses it — setting
`size_inferred` and logging a warning — and treats a record it cannot place as
unusable, falling back to the reference.

The undistorter itself never rescales:  `FisheyeUndistorter(cal,
calibrated_size, frame_size)` requires the two to be equal (Python:
`FisheyeUndistorter(cal, w, h, calibrated_size=(cw, ch))`). The legacy
`FisheyeUndistorter(cal, size)` form takes the params to be a 640x480
calibration, as it always has. A mismatch is a caller bug to fix — open the
camera at the calibrated size, or recalibrate at the capture size — not
something to paper over.

### Dual calibration slots (V2.11: firmware 1.3.1, or 1.3.0-rc4+)

V2.10 still had **one** record: storing a 1920x1080 calibration replaced the
640x480 one. V2.11 keeps two fixed slots, VGA (640x480) and HD (1920x1080), so a
unit can carry both and a camera at either resolution rectifies with its own
intrinsics.

| Call | What it does |
| --- | --- |
| `supports_fisheye_slots()` | Probes with the slot read (op `0x04`) at 640x480: firmware without slots rejects it as `InvalidParam`. Cached per `Calibration`; answered without a round trip once `supports_fisheye_size()` is known to be false. |
| `read_fisheye_slot(w, h)` | The record for 640x480 or 1920x1080 (anything else raises `RuntimeError` before sending), or `None`. V2.11: op `0x04`; the VGA slot may report size `0/0` when its params were written without one, HD always reports 1920x1080. V2.10: the single record if its recorded size matches. Older firmware: the legacy record, for 640x480 only. |
| `write_fisheye_sized(cal, w, h)` | Unchanged API. On V2.11 op `0x03` routes by size: 640x480 writes VGA (params + size, as on V2.10), 1920x1080 writes HD only; neither touches the other slot. |
| `clear_fisheye_slot(w, h)` | Erases one slot (op `0x05`); it then reads `None`. Clearing VGA drops its size record too; clearing an empty slot succeeds. Raises `RuntimeError` without sending on firmware without slots. |
| `resolve_fisheye_for(w, h)` | `resolve_fisheye()` for a camera at `w x h`. Reads that slot on V2.11; otherwise uses the single record only when its size (recorded or inferred) equals `w x h`. Anything else falls back to the reference, with `reason` saying why. |

**A record with no size counts as the VGA slot**, on every generation. That is
V2.11's own definition — ops `0x00`/`0x01`, the only way to write params without
a size, address the VGA slot, and op `0x02` reports it with `0/0` — so
`read_fisheye_slot(640, 480)` returns the same flash content the same way
whether the firmware is V2.11, V2.10 or older. The `0/0` keeps the size visibly
unconfirmed; `resolve_fisheye_for()` checks it against the principal point
(`infer_fisheye_cal_size()`) before trusting it.

**The 0.4.7 calls address the VGA slot on V2.11.** `read_fisheye()`,
`read_fisheye_record()` and the no-argument `resolve_fisheye()` all read VGA.
So after storing only an HD calibration, `read_fisheye_record()` returns `None`
(and with a VGA one stored, it returns that, not the HD record just written).
Confusing, but nothing is lost — read the HD slot with
`read_fisheye_slot(1920, 1080)`.

**Tighter plausibility.** V2.11 accepts a sized write only for 640x480 or
1920x1080, with every value finite, `fx, fy > 0`, and the principal point within
15% of the frame centre, inclusive: `|cx - w/2| <= 0.15*w`,
`|cy - h/2| <= 0.15*h` (640x480: cx 224..416, cy 168..312; 1920x1080: cx
672..1248, cy 378..702). `fisheye_cal_slot_plausible()` is that rule. The SDK
applies it host-side to **every** 640x480 / 1920x1080 sized write, whatever the
firmware, so the typical mistake — VGA params labelled 1920x1080, which V2.10's
"inside the frame" rule lets through — is refused with a message before
anything is sent. Other sizes are refused host-side on V2.11 only; V2.10 keeps
its looser rule (any plausible size up to 4096).

**Wrist undistortion picks the slot by camera size.** `install_wrist_undistorter()`
(`undistort_wrist=True`) now calls `resolve_fisheye_for()` with the camera's
configured width/height, so a 1080p camera gets the HD slot on V2.11. If the
unit has a usable calibration only for the *other* resolution, opening raises,
naming both sizes, rather than silently swapping in the approximate reference
(calibrate at the camera's size, or run the camera at the calibrated one). The
reference fallback with a warning is kept only for a unit with no usable
calibration at all, and only works on a 640x480 camera. A 1920x1080 camera on
such a unit still raises, because the reference is never rescaled.
`resolve_fisheye_for()` itself only reports a mismatch: it returns the reference
with a "belongs to AxB" `reason` and `mismatched_size = (A, B)`.

**Upgrading from V2.10 (rc1..rc3).** The single V2.10 record stays in the VGA
slot; it is never moved to HD. The VGA slot reports its size only when the size
record is exactly 640x480, so a 1920x1080 record written by rc1..rc3 reads with
size `0/0` (op `0x02` and `0x04` at 640x480). `read_fisheye_slot(1920, 1080)` is
then `None`, and `resolve_fisheye_for(640, 480)` infers 1920x1080 from the
principal point and reports it as `mismatched_size`. Wrist undistortion raises
on either camera size until the unit is recalibrated: on a 640x480 camera because
the calibration is for 1920x1080, and on a 1920x1080 camera because the HD slot
is empty. Store the calibration again with
`write_fisheye_sized(cal, 1920, 1080)` to put it in the HD slot.

```bash
python python/examples/fisheye_cal.py show                                   # lists both slots on V2.11
python python/examples/fisheye_cal.py clear-fisheye --width 1920 --height 1080
python python/examples/read_intrinsics.py left | jq .slots                   # per-slot JSON
```

Older-firmware facts the probes rely on: firmware up to 1.2.17 answers op
`0x02` with `InvalidParam` and a 37-byte op `0x03` with `LengthMismatch` (its
command table caps `0x2B` at 33 bytes); V2.10 (1.3.0-rc1..rc3) answers ops
`0x04`/`0x05` with `InvalidParam` (unknown op).
