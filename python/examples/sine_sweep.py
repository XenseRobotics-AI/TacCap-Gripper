#!/usr/bin/env python3
"""Sweep position-following torque, sine frequency and reference speed.

This moves hardware. Completion, Ctrl+C and errors leave the context by disabling
the motor. Plotting runs only after the connection has closed.
See docs/SINE_SWEEP.md for measurement definitions and examples.
"""
import argparse
import csv
import itertools
import json
import math
import random
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

SAMPLE_FIELDS = [
    "trial", "host_monotonic_s", "elapsed_s", "measured", "seq", "command_id",
    "ideal_opening", "last_command_opening", "expected_encoder_position",
    "position", "velocity_rad_s", "torque_nm", "temperature_c", "age_ms",
    "target_error_rad", "effective_torque_limit_nm", "target_limited", "ready", "fault",
]
COMMAND_FIELDS = [
    "trial", "tick", "host_monotonic_s", "elapsed_s", "opening", "api_call_ms", "skipped_ticks",
]


def number_list(text):
    try:
        values = [float(value) for value in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("use comma-separated numbers") from exc
    if not values or not all(math.isfinite(v) and v > 0 for v in values):
        raise argparse.ArgumentTypeError("values must be positive and finite")
    if len(set(values)) != len(values):
        raise argparse.ArgumentTypeError("duplicate sweep values are not allowed")
    return values


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("device", nargs="?", default="left")
    parser.add_argument("--torques", type=number_list, default=[.35, .6, .9, 1.2], help="Motor Nm budgets, comma-separated")
    parser.add_argument("--frequencies", type=number_list, default=[.25, .5, 1., 2., 3.], help="Sine frequencies in Hz")
    parser.add_argument("--speeds", type=number_list, default=[7.5], help="Motor reference slew limits in rad/s")
    parser.add_argument("--center", type=float, default=.5, help="Normalized opening center")
    parser.add_argument("--amplitude", type=float, default=.25, help="Normalized opening half amplitude")
    parser.add_argument("--hz", type=int, choices=(20, 25, 40, 50, 100), default=100, help="SDK feedback/control rate")
    parser.add_argument("--command-hz", type=float, default=100., help="Python position input rate")
    parser.add_argument("--cycles", type=int, default=8, help="Measured cycles per trial")
    parser.add_argument("--warmup-cycles", type=int, default=2, help="Initial cycles excluded from metrics")
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--seed", type=int, default=7, help="Seed for shuffled trial order")
    parser.add_argument("--rest", type=float, default=3., help="Seconds holding the start position before each trial")
    parser.add_argument("--positioning-torque", type=float, default=.35, help="Separate budget for positioning/rest, Nm")
    parser.add_argument("--settle-timeout", type=float, default=10.)
    parser.add_argument("--close-compensation-rad", type=float, default=.03)
    parser.add_argument("--max-temperature", type=float, default=60., help="Abort at this motor temperature, C")
    parser.add_argument("--note", default="", help="Load/object, mounting and ambient conditions")
    parser.add_argument("--output", type=Path,
                        default=Path("sine-runs") / datetime.now(timezone.utc).astimezone().strftime("%Y%m%d-%H%M%S"))
    parser.add_argument("--no-plot", action="store_true", help="Capture without optional plotting dependencies")
    args = parser.parse_args(argv)
    for name in ("center", "amplitude", "command_hz", "rest", "positioning_torque", "settle_timeout",
                 "close_compensation_rad", "max_temperature"):
        if not math.isfinite(getattr(args, name)):
            parser.error(f"{name} must be finite")
    if args.amplitude <= 0 or args.center - args.amplitude < 0 or args.center + args.amplitude > 1:
        parser.error("center +/- amplitude must remain in [0,1], with amplitude > 0")
    if max(args.torques + [args.positioning_torque]) > 1.2 or args.positioning_torque <= 0:
        parser.error("torque budgets must be in (0,1.2] Nm")
    if max(args.speeds) > 50:
        parser.error("reference speeds must be <= 50 rad/s")
    if not 0 < args.command_hz <= 500:
        parser.error("command-hz must be in (0,500]")
    if max(args.frequencies) * 20 > min(args.command_hz, args.hz):
        parser.error("each sine cycle requires at least 20 configured command and feedback samples")
    if args.cycles < 2 or args.warmup_cycles < 0 or args.repeats < 1:
        parser.error("cycles >= 2, warmup-cycles >= 0 and repeats >= 1 are required")
    if args.rest < 0 or args.settle_timeout <= 0 or not 0 <= args.close_compensation_rad <= .05:
        parser.error("rest >= 0, settle-timeout > 0 and compensation in [0,0.05] are required")
    if not 0 < args.max_temperature <= 80:
        parser.error("max-temperature must be in (0,80] C")
    return args


def opening_at(elapsed, frequency, center, amplitude):
    # Cosine is a sine with a phase shift: start at the open end at zero slope.
    return center + amplitude * math.cos(2 * math.pi * frequency * elapsed)


def encoder_target(opening, travel, compensation):
    # Feedback uses the ORIGINAL calibrated 0..1 coordinate and clips at zero.
    return min(1., max(0., opening - compensation / travel * (1 - opening)))


def make_trials(args, travel):
    trials = []
    for repeat, torque, frequency, speed in itertools.product(
            range(1, args.repeats + 1), args.torques, args.frequencies, args.speeds):
        required_speed = 2 * math.pi * frequency * args.amplitude * (travel + args.close_compensation_rad)
        trials.append({"repeat": repeat, "torque_limit_nm": torque, "frequency_hz": frequency,
                           "speed_rad_s": speed, "required_peak_speed_rad_s": required_speed,
                           "reference_speed_limited": required_speed > speed})
    random.Random(args.seed).shuffle(trials)
    for index, trial in enumerate(trials, 1):
        trial.update(id=index, status="pending")
    return trials


def check_state(state, max_temperature):
    if state.fault:
        raise RuntimeError(f"Gripper fault: {state.fault}")
    values = (state.position, state.velocity_rad_s, state.torque_nm, state.temperature_c, state.age_ms)
    if not all(math.isfinite(v) for v in values):
        raise RuntimeError("Non-finite motor feedback")
    if state.age_ms > 200:
        raise RuntimeError(f"Stale motor feedback: {state.age_ms:.1f} ms")
    if state.temperature_c >= max_temperature:
        raise RuntimeError(f"Temperature stop: {state.temperature_c:.1f} C >= {max_temperature:g} C")


def prepare(gripper, args):
    gripper.move(args.center + args.amplitude, torque_limit_nm=args.positioning_torque, speed_rad_s=.5)
    deadline = time.monotonic() + args.settle_timeout
    settled_since = None
    while True:
        state = gripper.state
        check_state(state, args.max_temperature)
        now = time.monotonic()
        # Use the compensated motor error, not the original normalized opening.
        if (state.ready and state.target_error_rad is not None and
                abs(state.target_error_rad) <= .015 and abs(state.velocity_rad_s) <= .05):
            if settled_since is None:
                settled_since = now
            if now - settled_since >= .2:
                break
        else:
            settled_since = None
        if now >= deadline:
            raise RuntimeError(f"Cannot settle at start position: {state}")
        time.sleep(.01)
    deadline = time.monotonic() + args.rest
    while time.monotonic() < deadline:
        check_state(gripper.state, args.max_temperature)
        time.sleep(min(.02, max(0., deadline - time.monotonic())))


def record_trial(gripper, args, trial, sample_writer, command_writer, *, clock=time.monotonic, sleep=time.sleep):
    """Single timed loop; native SDK controls independently. Missed ticks are skipped."""
    frequency = trial["frequency_hz"]
    duration = (args.warmup_cycles + args.cycles) / frequency
    timing_before = gripper.timing
    trial["timing_before"] = asdict(timing_before)
    trial["start_temperature_c"] = gripper.state.temperature_c
    started = clock()
    trial["start_monotonic_s"] = started
    next_command = next_sample = started
    last_tick = -1
    last_seq = gripper.state.seq
    last_command = args.center + args.amplitude
    samples = commands = skipped = 0
    trial["status"] = "running"
    try:
        while clock() < started + duration:
            now = clock()
            if now >= next_command:
                tick = max(last_tick + 1, math.floor((now - started) * args.command_hz))
                missed = max(0, tick - last_tick - 1)
                last_command = opening_at(now - started, frequency, args.center, args.amplitude)
                gripper.move(last_command, torque_limit_nm=trial["torque_limit_nm"],
                             speed_rad_s=trial["speed_rad_s"], wait=False)
                completed = clock()
                command_writer.writerow([trial["id"], tick, now, now - started, last_command,
                                         (completed - now) * 1000, missed])
                commands += 1
                skipped += missed
                last_tick = tick
                next_command = started + (tick + 1) / args.command_hz
            now = clock()
            if now >= next_sample:
                state = gripper.state
                sampled = clock()
                elapsed = sampled - started
                ideal = opening_at(elapsed, frequency, args.center, args.amplitude)
                if state.seq != last_seq:
                    sample_writer.writerow([
                        trial["id"], sampled, elapsed, int(elapsed >= args.warmup_cycles / frequency),
                        state.seq, state.command_id, ideal, last_command,
                        encoder_target(ideal, gripper.info.motor_travel_rad, args.close_compensation_rad),
                        state.position, state.velocity_rad_s, state.torque_nm, state.temperature_c,
                        state.age_ms, state.target_error_rad, state.effective_torque_limit_nm,
                        state.target_limited, state.ready, state.fault,
                    ])
                    samples += 1
                    last_seq = state.seq
                check_state(state, args.max_temperature)
                # Poll twice per configured feedback period, only retain new sequences.
                next_sample = max(next_sample + 1 / (args.hz * 2),
                                  started + (math.floor(elapsed * args.hz * 2) + 1) / (args.hz * 2))
            sleep(max(0., min(next_command, next_sample, started + duration) - clock()))
        trial["status"] = "complete"
    except BaseException as exc:
        trial.update(status="aborted", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        elapsed = max(clock() - started, 1e-9)
        after = gripper.timing
        trial.update(duration_s=elapsed, samples=samples, api_commands=commands, skipped_command_ticks=skipped,
                     observed_sample_hz=samples / elapsed, api_command_hz=commands / elapsed,
                     native_feedback_hz=(after.feedback_frames - timing_before.feedback_frames) / elapsed,
                     native_command_hz=(after.command_frames - timing_before.command_frames) / elapsed,
                     timing_after=asdict(after))


def write_metadata(path, metadata):
    path.write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")


def main(argv=None):
    args = parse_args(argv)
    # A new directory is required; never overwrite a previous experiment.
    args.output.mkdir(parents=True, exist_ok=False)
    settings = vars(args).copy()
    settings["output"] = str(args.output.resolve())
    metadata = {"schema_version": 1, "simulated": False, "created_utc": datetime.now(timezone.utc).isoformat(),
                    "settings": settings, "status": "initializing", "trials": []}
    run_path = args.output / "run.json"
    write_metadata(run_path, metadata)
    try:
        from xense.taccap import Gripper, __version__

        metadata["sdk_version"] = __version__
        with Gripper(args.device, torque_limit_nm=args.positioning_torque, speed_rad_s=.5,
                     update_hz=args.hz, close_compensation_rad=args.close_compensation_rad) as gripper:
            metadata["device"] = asdict(gripper.info)
            if gripper.info.firmware_version < (1, 1, 8):
                raise RuntimeError("This benchmark requires firmware >= 1.1.8 execution telemetry")
            travel = gripper.info.motor_travel_rad
            metadata["trials"] = make_trials(args, travel)
            duration = sum((args.cycles + args.warmup_cycles) / t["frequency_hz"] + args.rest for t in metadata["trials"])
            print(f"{len(metadata['trials'])} trials, about {duration / 60:.1f} min plus positioning; output: {args.output}")
            print(f"Opening range {args.center - args.amplitude:g}..{args.center + args.amplitude:g}, "
                  f"compensation {args.close_compensation_rad:g} rad; start/rest budget {args.positioning_torque:g} Nm")
            for frequency in args.frequencies:
                required_speed = next(t["required_peak_speed_rad_s"] for t in metadata["trials"]
                                      if t["frequency_hz"] == frequency)
                print(f"Sine {frequency:g} Hz needs {required_speed:.3f} rad/s peak reference speed "
                      f"with measured travel {travel:.3f} rad")
            limited = [t["id"] for t in metadata["trials"] if t["reference_speed_limited"]]
            if limited:
                print(f"Reference speed is below sine peak demand in trials {limited}; these will be flagged in plots.")
            print("This will move the gripper repeatedly. Clear the motion area; support any load.")
            input("Press Enter to start; Ctrl+C cancels. ")
            gripper.start()
            metadata["status"] = "running"
            with (args.output / "samples.csv").open("w", newline="") as samples_file, \
                    (args.output / "commands.csv").open("w", newline="") as commands_file:
                samples = csv.writer(samples_file)
                commands = csv.writer(commands_file)
                samples.writerow(SAMPLE_FIELDS)
                commands.writerow(COMMAND_FIELDS)
                for trial in metadata["trials"]:
                    print(f"Trial {trial['id']}: {trial['torque_limit_nm']:g} Nm, "
                          f"{trial['frequency_hz']:g} Hz, {trial['speed_rad_s']:g} rad/s")
                    trial["status"] = "positioning"
                    write_metadata(run_path, metadata)
                    prepare(gripper, args)
                    record_trial(gripper, args, trial, samples, commands)
                    # Cancel the periodic target before writing/reporting between trials.
                    gripper.move(args.center + args.amplitude, torque_limit_nm=args.positioning_torque, speed_rad_s=.5)
                    samples_file.flush()
                    commands_file.flush()
                    write_metadata(run_path, metadata)
        metadata["status"] = "complete"
    except BaseException as exc:
        metadata.update(status="aborted", error=f"{type(exc).__name__}: {exc}")
        for trial in metadata["trials"]:
            if trial["status"] in ("positioning", "running"):
                trial.update(status="aborted", error=metadata["error"])
        raise
    finally:
        metadata["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_metadata(run_path, metadata)
        print(f"Capture status: {metadata['status']}. Saved to {args.output.resolve()}")
    if not args.no_plot:
        try:
            from plot_sine_sweep import plot_run
            plot_run(args.output)
        except ImportError as exc:
            print(f"Plotting dependency unavailable ({exc}). CSV capture is complete.")
            print("Install python/examples/requirements-plot.txt, then run plot_sine_sweep.py on this directory.")


if __name__ == "__main__":
    main()
