#!/usr/bin/env python3
"""Measure application calls, native control writes and feedback independently.

Default: enable and hold the current position. --motion sine moves the jaws.
Exit disables the motor. --plot RUN only reads saved results and needs no SDK.
"""
import argparse
import csv
import itertools
import json
import math
import platform
import statistics
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from sine_sweep import check_state, number_list, opening_at

COMMAND_FIELDS = ["elapsed_s", "position", "call_ms", "late_ms", "skipped_ticks"]
READ_FIELDS = ["elapsed_s", "call_ms", "late_ms", "skipped_ticks", "seq", "new_feedback",
               "unobserved_updates", "age_ms", "position", "target", "velocity_rad_s",
               "torque_nm", "temperature_c", "target_error_rad", "effective_torque_limit_nm"]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device", nargs="?", default="left")
    parser.add_argument("--sdk-hz", type=number_list, default=[20, 50, 100])
    parser.add_argument("--command-hz", type=number_list, default=[50, 100, 200])
    parser.add_argument("--read-hz", type=number_list, default=[500])
    parser.add_argument("--duration", type=float, default=10.)
    parser.add_argument("--warmup", type=float, default=1.)
    parser.add_argument("--motion", choices=("hold", "sine"), default="hold")
    parser.add_argument("--sine-hz", type=float, default=.5)
    parser.add_argument("--center", type=float, default=.5)
    parser.add_argument("--amplitude", type=float, default=.2)
    parser.add_argument("--torque", type=float, default=.6, help="Motor torque budget in Nm")
    parser.add_argument("--speed", type=float, default=7.5, help="Reference slew limit in rad/s")
    parser.add_argument("--max-temperature", type=float, default=60.)
    parser.add_argument("--note", default="", help="Host load, USB setup and mechanical load")
    parser.add_argument("--output", type=Path, default=Path("frequency-runs") /
                        datetime.now(timezone.utc).astimezone().strftime("%Y%m%d-%H%M%S"))
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--plot", type=Path, metavar="RUN", help="Only plot a saved run")
    args = parser.parse_args(argv)
    if args.plot:
        return args
    if any(v not in (20, 25, 40, 50, 100) for v in args.sdk_hz):
        parser.error("sdk-hz must contain 20,25,40,50,100")
    if max(args.command_hz) > 500 or max(args.read_hz) > 2000:
        parser.error("command-hz <= 500 and read-hz <= 2000 are required")
    for key in ("duration", "warmup", "sine_hz", "center", "amplitude", "torque", "speed", "max_temperature"):
        if not math.isfinite(getattr(args, key)):
            parser.error(f"{key} must be finite")
    if not 1 <= args.duration <= 300 or not 0 <= args.warmup <= 30:
        parser.error("duration must be 1..300 s; warmup must be 0..30 s")
    if not 0 < args.torque <= 1.2 or not 0 < args.speed <= 50:
        parser.error("torque must be (0,1.2] Nm; speed must be (0,50] rad/s")
    if not 0 < args.max_temperature <= 80:
        parser.error("max-temperature must be (0,80] C")
    if args.amplitude <= 0 or not 0 <= args.center - args.amplitude < args.center + args.amplitude <= 1:
        parser.error("center +/- amplitude must lie in [0,1]")
    if args.sine_hz <= 0 or (args.motion == "sine" and
                            args.sine_hz * 20 > min(args.sdk_hz + args.command_hz)):
        parser.error("sine requires a positive frequency and >= 20 SDK/command samples per cycle")
    return args


def distribution(values):
    """Linear-interpolated percentiles; empty measurements remain null."""
    if not values:
        return None
    ordered = sorted(values)

    def percentile(p):
        index = (len(ordered) - 1) * p
        low = int(index)
        high = min(low + 1, len(ordered) - 1)
        return ordered[low] + (ordered[high] - ordered[low]) * (index - low)

    return {"mean": statistics.mean(values), "std": statistics.pstdev(values),
            "p50": percentile(.5), "p95": percentile(.95), "p99": percentile(.99), "max": max(values)}


def advance(deadline, completed, hz):
    """Skip expired slots instead of bursting to catch up after a slow call."""
    skipped = max(0, math.floor((completed - deadline) * hz))
    return deadline + (skipped + 1) / hz, skipped


def record_trial(gripper, args, trial, commands, reads, *, clock=time.monotonic, sleep=time.sleep):
    last_seq = gripper.state.seq
    before = gripper.timing
    started = clock()
    next_command = next_read = started
    trial.update(status="running", timing_before=asdict(before), start_monotonic_s=started)
    try:
        while clock() < started + args.duration:
            now = clock()
            if now >= next_command:
                position = (trial["hold_position"] if args.motion == "hold" else
                            opening_at(now - started, args.sine_hz, args.center, args.amplitude))
                gripper.set_position(position)
                completed = clock()
                deadline, skipped = advance(next_command, completed, trial["command_hz"])
                commands.append(dict(zip(COMMAND_FIELDS, [now - started, position, (completed - now) * 1000,
                                                         (now - next_command) * 1000, skipped])))
                next_command = deadline
            now = clock()
            if now >= started + args.duration:
                break
            if now >= next_read:
                state = gripper.state
                completed = clock()
                deadline, skipped = advance(next_read, completed, trial["read_hz"])
                # seq is a host feedback counter. Gaps are unseen cache updates, not proven wire loss.
                if state.seq < last_seq:
                    raise RuntimeError("Feedback sequence reset inside an active trial")
                new = int(state.seq != last_seq)
                unobserved = max(0, state.seq - last_seq - 1)
                reads.append(dict(zip(READ_FIELDS, [now - started, (completed - now) * 1000,
                    (now - next_read) * 1000, skipped, state.seq, new, unobserved, state.age_ms,
                    state.position, state.target, state.velocity_rad_s, state.torque_nm, state.temperature_c,
                    state.target_error_rad, state.effective_torque_limit_nm])))
                last_seq = state.seq
                check_state(state, args.max_temperature)
                next_read = deadline
            sleep(max(0., min(next_command, next_read, started + args.duration) - clock()))
        trial["status"] = "complete"
    except BaseException as exc:
        trial.update(status="aborted", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        # Read cached counters before closing the connection; no diagnostic UART traffic in the loop.
        after = gripper.timing
        elapsed = max(clock() - started, 1e-9)
        trial.update(duration_s=elapsed, timing_after=asdict(after))
        trial["metrics"] = summarize(commands, reads, before, after, elapsed)


def summarize(commands, reads, before, after, elapsed):
    result = {"api_command_hz": len(commands) / elapsed, "api_read_hz": len(reads) / elapsed,
              "native_command_hz": (after.command_frames - before.command_frames) / elapsed,
              "native_feedback_hz": (after.feedback_frames - before.feedback_frames) / elapsed,
              "observed_feedback_hz": sum(r["new_feedback"] for r in reads) / elapsed,
              "duplicate_reads": sum(not r["new_feedback"] for r in reads),
              "unobserved_cache_updates": sum(r["unobserved_updates"] for r in reads),
              "feedback_age_ms": distribution([r["age_ms"] for r in reads])}
    for name, rows in (("command", commands), ("read", reads)):
        result[f"{name}_skipped_ticks"] = sum(r["skipped_ticks"] for r in rows)
        result[f"{name}_call_ms"] = distribution([r["call_ms"] for r in rows])
        result[f"{name}_late_ms"] = distribution([r["late_ms"] for r in rows])
        result[f"{name}_interval_ms"] = distribution([
            (b["elapsed_s"] - a["elapsed_s"]) * 1000 for a, b in itertools.pairwise(rows)])
    # Detection intervals include Python polling jitter and are not native UART arrival intervals.
    fresh = [r for r in reads if r["new_feedback"]]
    result["observed_feedback_interval_ms"] = distribution([
        (b["elapsed_s"] - a["elapsed_s"]) * 1000 for a, b in itertools.pairwise(fresh)])
    return result


def save_trial(directory, trial, commands, reads):
    for name, fields, rows in (("commands", COMMAND_FIELDS, commands), ("reads", READ_FIELDS, reads)):
        with (directory / f"trial-{trial['id']:03d}-{name}.csv").open("w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)


def plot_run(directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = json.loads((directory / "run.json").read_text())
    trials = [t for t in data["trials"] if t.get("metrics")]
    if not trials:
        raise ValueError("No measured trials in this run")
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    labels = [f"T{t['id']}\n{t['sdk_hz']}/{t['command_hz']:g}/{t['read_hz']:g}" for t in trials]
    for key, label in (("api_command_hz", "Python target calls"), ("api_read_hz", "Python state reads"),
                       ("native_command_hz", "Native writes"), ("native_feedback_hz", "Native feedback"),
                       ("observed_feedback_hz", "Unique feedback observed")):
        axes[0, 0].plot(labels, [t["metrics"][key] for t in trials], marker="o", label=label)
    axes[0, 0].set(ylabel="Hz", title="Measured rates (labels: SDK / command / read Hz)")
    axes[0, 0].legend(fontsize=8)
    for key, label in (("command_call_ms", "Target call p95"), ("read_call_ms", "State read p95"),
                       ("feedback_age_ms", "Feedback age p95")):
        axes[0, 1].plot(labels, [(t["metrics"][key] or {}).get("p95", float("nan")) for t in trials],
                        marker="o", label=label)
    axes[0, 1].set(ylabel="ms", title="Host API duration and feedback freshness")
    axes[0, 1].legend(fontsize=8)
    for trial in trials:
        with (directory / f"trial-{trial['id']:03d}-reads.csv").open() as source:
            rows = list(csv.DictReader(source))
        times = [float(r["elapsed_s"]) for r in rows if int(r["new_feedback"])]
        intervals = [(b - a) * 1000 for a, b in itertools.pairwise(times)]
        if intervals:
            axes[1, 0].hist(intervals, bins=40, histtype="step", label=f"T{trial['id']}")
    axes[1, 0].set(xlabel="ms (includes polling jitter)", ylabel="Count", title="Observed fresh-feedback intervals")
    axes[1, 0].legend(fontsize=8)
    width = .35
    x = list(range(len(trials)))
    axes[1, 1].bar([v - width / 2 for v in x],
                   [t["metrics"]["command_skipped_ticks"] for t in trials], width, label="Command slots skipped")
    axes[1, 1].bar([v + width / 2 for v in x],
                   [t["metrics"]["read_skipped_ticks"] for t in trials], width, label="Read slots skipped")
    axes[1, 1].set_xticks(x, labels)
    axes[1, 1].set(ylabel="Count", title="Host scheduler overruns (not wire loss)")
    axes[1, 1].legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    source = "SIMULATED | " if data.get("simulated") else ""
    fig.suptitle(f"TacCap frequency test | {source}{data['status']} | {data['settings']['motion']}")
    for extension in ("png", "svg"):
        fig.savefig(directory / f"frequency-summary.{extension}", dpi=160)
    plt.close(fig)
    print(f"Plots: {directory / 'frequency-summary.png'}")


def main(argv=None):
    args = parse_args(argv)
    if args.plot:
        plot_run(args.plot)
        return
    args.output.mkdir(parents=True, exist_ok=False)
    settings = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    data = {"schema_version": 1, "simulated": False, "status": "initializing", "settings": settings,
            "created_utc": datetime.now(timezone.utc).isoformat(), "platform": platform.platform(),
            "python": platform.python_version(), "trials": []}
    try:
        from xense.taccap import Gripper, __version__
        data["sdk_version"] = __version__
        rates = list(itertools.product(args.sdk_hz, args.command_hz, args.read_hz))
        print(f"{len(rates)} trials; {args.motion}; {args.duration:g} s per trial. Output: {args.output}")
        print("Enables the motor; sine mode moves the jaws. Exit disables and may release a load.")
        input("Clear the motion area and support any load. Press Enter to start; Ctrl+C cancels. ")
        data["status"] = "running"
        for index, (sdk_hz, command_hz, read_hz) in enumerate(rates, 1):
            trial = {"id": index, "sdk_hz": int(sdk_hz), "command_hz": command_hz,
                     "read_hz": read_hz, "status": "initializing"}
            data["trials"].append(trial)
            commands, reads = [], []
            try:
                with Gripper(args.device, update_hz=int(sdk_hz), torque_limit_nm=args.torque,
                             speed_rad_s=args.speed) as gripper:
                    trial["device"] = asdict(gripper.info)
                    gripper.start()
                    state = gripper.state
                    check_state(state, args.max_temperature)
                    trial["hold_position"] = state.position
                    if args.motion == "sine":
                        gripper.move(args.center + args.amplitude, wait=True, timeout=15.)
                    end = time.monotonic() + args.warmup
                    while time.monotonic() < end:
                        check_state(gripper.state, args.max_temperature)
                        time.sleep(.01)
                    record_trial(gripper, args, trial, commands, reads)
            except BaseException as exc:
                trial.update(status="aborted", error=f"{type(exc).__name__}: {exc}")
                raise
            finally:
                save_trial(args.output, trial, commands, reads)
                (args.output / "run.json").write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
            m = trial["metrics"]
            print(f"T{index}: calls={m['api_command_hz']:.1f}, reads={m['api_read_hz']:.1f}, "
                  f"native writes={m['native_command_hz']:.1f}, native feedback={m['native_feedback_hz']:.1f} Hz")
        data["status"] = "complete"
    except BaseException as exc:
        data.update(status="aborted", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        data["finished_utc"] = datetime.now(timezone.utc).isoformat()
        (args.output / "run.json").write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
        print(f"Saved {data['status']} run: {args.output.resolve()}")
    if not args.no_plot:
        try:
            plot_run(args.output)
        except ImportError:
            print("Install python/examples/requirements-plot.txt, then use --plot RUN to create charts.")


if __name__ == "__main__":
    main()
