# 标定

<!-- 从 README.md 拆出，保持内容不变；README 只保留入门路径。 -->

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
| Fisheye camera `fx, fy, cx, cy, k1..k4` | `0x2B` | leader + follower | `read_fisheye()` / `write_fisheye()` |
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

C++ returns a `ResolvedFisheyeCal` (`calibration`, `is_reference`, `reason`);
Python returns the same three as a plain tuple:

```python
cal, is_reference, reason = g.calibration.resolve_fisheye()
if is_reference:
    log.warning("rectifying with the SDK's reference intrinsics: %s", reason)
```

`reason` is non-empty exactly when `is_reference` is true.

Three ways a unit fails to supply its own calibration, all of which fall back:

| What happened | How it is detected |
| --- | --- |
| The lens was never calibrated | `read_fisheye()` → `None` (`CalNotSet`) |
| Firmware answered with an all-zero record | `!is_usable_fisheye_cal()` — see above |
| Firmware predates command set V2.0 | `ProtocolError` (`InvalidCmd`) on `0x2B` |

What stands in is `FISHEYE_FALLBACK_CAL`, a reference calibration measured on a
sample TC-GU-01 and compiled into the SDK
(`cpp/include/taccap/components/fisheye_undistorter.hpp`):

```text
fx 213.0303  fy 212.7928   cx 321.4000  cy 239.9500
k1  -0.0172  k2   0.0091   k3  -0.0146  k4   0.0051
```

Every unit carries the same lens on the same 640x480 sensor, so these shared
numbers are much closer to correct than no rectification at all — which is why
`install_wrist_undistorter()` now installs undistortion **always**, and logs
which of the two calibrations it used.

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

The one case that still throws rather than falling back is a camera running at
something other than the calibrated 640x480: the firmware record holds only the
8 intrinsic/distortion floats and no image size, so serving another resolution
would mean guessing a scale factor and rectifying wrongly without a trace. That
is a caller bug to fix, not something to paper over.
