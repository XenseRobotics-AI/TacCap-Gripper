# Updating the follower's motor firmware

> Chinese version: [MOTOR_OTA-zh.md](MOTOR_OTA-zh.md)

The follower gripper carries a RobStride motor that runs its own firmware,
separate from the gripper's MCU firmware. This guide updates the motor firmware
over the gripper's USB-C cable with `python/examples/motor_ota_update.py`. The
motor stays mounted.

| Motor | Required motor firmware |
| --- | --- |
| EL05 | ≥ 1.0.5.0.4 |

## Before you start

- **Gripper firmware 1.2.16** (at least 1.2.8 for motor updates). Check it with
  `follower_status.py` (the `[fw]` line). If it is older, update the gripper
  firmware first with `ota_update.py` and power-cycle; a gripper firmware update
  also resets the motor to its normal protocol, so it must come before the motor
  update, never after.
- **SDK 0.4.3 or newer**, installed so that `import xense.taccap` works.
- **The motor firmware image** ships in [`firmware/`](../firmware/) (listed under
  `motors` in `manifest.json`). The script picks the one for the motor model
  recorded on the gripper and checks its CRC32. To flash a different image, put
  it before the target (`motor_ota_update.py EL05_1.0.5.0.4.bin left`) and keep
  the file name as delivered: the script reads the motor model and version from
  it and checks them against the image and the gripper.
- **Nothing else using the gripper.** Stop your own programs first.
- **Jaws clear.** After each power cycle the gripper opens and closes once to
  calibrate.

"Power-cycle" below always means: unplug the gripper's **24 V power cable** for
about 2 seconds and plug it back. The USB cable can stay connected; the MCU and
the motor both run from 24 V.

## Why the command runs twice

Our SDK drives the motor on the **MIT protocol**. The motor accepts firmware
updates only on its **private protocol**, and a protocol change takes effect at
the next power-up. So one update is:

1. run the script: it switches the motor to the private protocol;
2. power-cycle;
3. run the same command again: it flashes, verifies, records the new version in
   the gripper, sets the motion envelope to the motor's rating if it differs, and
   switches the motor back to MIT;
4. power-cycle.

The private protocol is used only for the update. The gripper is back on MIT,
ready for normal use, at the end.

## Steps

The examples assume the left follower; use `right` or the full serial number
(for example `TCGU01A25Z0017s`) for another one.

### 1. Check the current state

```bash
python python/examples/follower_status.py left
```

Look at `[fw]` (gripper firmware) and `[motor fw]` (motor firmware). The motor
firmware may show as unreadable at this point; that is expected while the motor
is on MIT and nothing has been recorded yet.

### 2. First run: switch to the private protocol

```bash
python python/examples/motor_ota_update.py left
```

```text
=== motor OTA, step 1 of 2 ===
  the motor is on the MIT protocol and does not answer OTA frames there,
  so it has to be switched to its private protocol first.
Switch the motor to the private protocol? type 'yes': yes

done. Now unplug the gripper's 24 V power cable for ~2 s and plug it back (USB may stay in),
wait ~10 s, then run the same command again to flash.
```

**Power-cycle** and wait about 10 seconds. On the private protocol the motor
stays disabled and the gripper does not calibrate; that is expected.

### 3. Second run: flash

```bash
python python/examples/motor_ota_update.py left
```

```text
=== motor OTA, step 2 of 2 ===
  gripper      : TCGU01A25Z0017s  firmware FirmwareVersion(1.2.16.0)
  motor        : EL05  can_id 1  uid ...
  motor fw now : 1.0.5.0.2
  image        : .../TacCap-Gripper/firmware/EL05_1.0.5.0.4.bin
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

Flashing takes about half a minute. Warnings like `motor OTA start: no reply
(attempt 1/6)` at the start are normal: the motor restarts into its bootloader
first and answers on a later attempt.

**Motion envelope.** If the gripper's stored motion envelope (its torque and
thermal protection) differs from the motor's rating, the script sets it to the
rating after flashing (EL05: cont 1.1 / peak 1.8 N·m) and says so before you
confirm:

```text
  envelope     : stored cont 1.000 / peak 2.000 N*m, will be set to cont 1.100 / peak 1.800 after flashing
```

The travel calibration and open direction are kept. Pass `--keep-envelope` to
leave the envelope as it is.

### 4. Power-cycle and check

**Power-cycle** and let the gripper calibrate (about 10 seconds), then:

```bash
python python/examples/follower_status.py left
```

`[motor fw]` should show the new version, for example
`MotorVersion(1.0.5.0.4, flash: host-written)`. The version is recorded in the
gripper, so it stays readable on MIT from now on.

## If something goes wrong

| Symptom | What to do |
| --- | --- |
| `no recorded motor model` / `no shipped image for motor ...` | The gripper's motor model is not recorded, or no image for it ships in `firmware/`. Pass the image explicitly, or set the model first ([firmware/README.md](../firmware/README.md#follower-motor-setup)). |
| `... has CRC32 ..., firmware/manifest.json says ...` | The file in `firmware/` is damaged or was replaced. Restore it with `git checkout firmware/`. |
| `cannot tell the motor model from ...` | The image file was renamed. Restore the original name, or pass `--model EL05`. |
| `this gripper records a ... motor, the image is for ...` | The image is for a different motor model. Do not override this; contact us. |
| `follower firmware ... lacks the 0x5B relay` | Update the gripper firmware first (see *Before you start*). |
| `motor OTA ...: no reply after N attempts` | Power-cycle and run the same command again. An interrupted update restarts from the beginning; the motor is not damaged. |
| `!! expected ...` after flashing | Power-cycle and run the same command again. |
| The script stopped and the gripper does not calibrate any more | The motor is still on the private protocol. Run `python python/examples/motor_ota_update.py --to-mit left`, then power-cycle. |

To switch the motor back to MIT at any time without flashing:

```bash
python python/examples/motor_ota_update.py --to-mit left
```

then power-cycle.

## Doing it from your own code

The script is a thin wrapper around `xense.taccap.MotorOtaSession`:

```python
import xense.taccap as t

g = t.FollowerGripper(t.find_left().mcu_device)
m = g.motor
# the motor must already be on the private protocol (see step 2)
session = t.MotorOtaSession(m, m.get_can_id())
session.preflight("EL05")                       # private protocol + recorded model
session.update_from_file("firmware/EL05_1.0.5.0.4.bin", lambda done, total: None)
m.set_motor_fw_version("1.0.5.0.4")             # record it for MIT reads
m.switch_protocol(t.MotorProtocol.Mit)          # then power-cycle
```
