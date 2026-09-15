#!/usr/bin/env python3
"""Plot a sine_sweep.py capture, or generate a clearly labeled synthetic demo.

Usage:
  python python/examples/plot_sine_sweep.py sine-runs/my-test
  python python/examples/plot_sine_sweep.py --demo /tmp/gripper-sine-demo
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def read_csv(path):
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


def column(rows, name):
    return np.array([float(row[name]) if row.get(name) not in (None, "") else np.nan for row in rows])


def time_weights(t):
    dt = np.diff(t)
    if len(t) < 2 or not np.all(dt > 0):
        raise ValueError("Feedback timestamps must be strictly increasing within each trial")
    weights = np.r_[dt[0], dt[:-1] + dt[1:], dt[-1]] / 2
    return weights / weights.sum()


def fit_sine(t, values, frequency, weights):
    design = np.column_stack([np.ones_like(t), np.cos(2 * np.pi * frequency * t),
                              np.sin(2 * np.pi * frequency * t)])
    coefficients = np.linalg.lstsq(design * np.sqrt(weights[:, None]), values * np.sqrt(weights), rcond=None)[0]
    amplitude = float(np.hypot(coefficients[1], coefficients[2]))
    phase = float(np.arctan2(coefficients[2], coefficients[1]))
    variance = np.sum(weights * (values - np.sum(weights * values)) ** 2)
    r2 = float(1 - np.sum(weights * (values - design @ coefficients) ** 2) / variance) if variance > 1e-12 else None
    return amplitude, phase, r2


def summarize(trial, rows, commands, settings):
    measured = [row for row in rows if row["measured"] == "1" and not row["fault"]
                and float(row["elapsed_s"]) < (settings["warmup_cycles"] + settings["cycles"]) / trial["frequency_hz"]]
    result = {key: trial[key] for key in ("id", "repeat", "torque_limit_nm", "frequency_hz", "speed_rad_s",
                                         "required_peak_speed_rad_s", "reference_speed_limited", "status")}
    for key in ("native_feedback_hz", "native_command_hz", "api_command_hz", "skipped_command_ticks"):
        result[key] = trial.get(key)
    result.update(samples=len(measured), fit_reliable=False)
    if len(measured) < 10:
        result["quality_note"] = "insufficient measured samples"
        return result
    t = column(measured, "elapsed_s")
    target = column(measured, "expected_encoder_position")
    position = column(measured, "position")
    if not np.all(np.isfinite(np.r_[t, target, position])):
        raise ValueError(f"Trial {trial['id']}: non-finite position data")
    weights = time_weights(t)
    error = target - position
    target_amp, target_phase, _ = fit_sine(t, target, trial["frequency_hz"], weights)
    actual_amp, actual_phase, r2 = fit_sine(t, position, trial["frequency_hz"], weights)
    coverage = (t[-1] - t[0]) / (settings["cycles"] / trial["frequency_hz"])
    max_gap = float(np.max(np.diff(t)))
    reliable = (trial["status"] == "complete" and coverage >= .95 and
                max_gap * trial["frequency_hz"] <= .1 and r2 is not None and r2 >= .8 and
                actual_amp > 1e-4 and target_amp > 1e-4 and np.min(target) > 0 and np.max(target) < 1)
    lag = (actual_phase - target_phase + np.pi) % (2 * np.pi) - np.pi
    torque = column(measured, "torque_nm")
    temperature = column(measured, "temperature_c")
    envelope = column(measured, "effective_torque_limit_nm")
    finite_envelope = envelope[np.isfinite(envelope)]
    result.update(
        rmse_opening=float(np.sqrt(np.sum(weights * error ** 2))),
        max_abs_error_opening=float(np.max(np.abs(error))),
        bias_opening=float(np.sum(weights * error)),
        amplitude_ratio=actual_amp / target_amp if target_amp > 1e-4 else None,
        apparent_phase_lag_deg=float(np.degrees(lag)) if reliable else None,
        apparent_phase_lag_ms=float(lag / (2 * np.pi * trial["frequency_hz"]) * 1000) if reliable else None,
        sine_fit_r2=r2, fit_reliable=reliable,
        quality_note="" if reliable else "phase omitted: incomplete/sparse capture, distorted/stationary output or endpoint clipping",
        measured_coverage=float(coverage), observed_sample_hz=float((len(t) - 1) / (t[-1] - t[0])),
        max_sample_gap_ms=max_gap * 1000, max_age_ms=float(np.max(column(measured, "age_ms"))),
        torque_rms_nm=float(np.sqrt(np.sum(weights * torque ** 2))),
        peak_abs_torque_nm=float(np.max(np.abs(torque))),
        temperature_start_c=float(temperature[0]), temperature_end_c=float(temperature[-1]),
        temperature_max_c=float(np.max(temperature)),
        minimum_firmware_envelope_nm=float(np.min(finite_envelope)) if len(finite_envelope) else None,
        mcu_limited_fraction=float(np.sum(weights * np.array([r["target_limited"] in ("True", "1") for r in measured]))),
    )
    if commands:
        result["api_call_p95_ms"] = float(np.percentile(column(commands, "api_call_ms"), 95))
        result["api_call_max_ms"] = float(np.max(column(commands, "api_call_ms")))
    return result


def plot_run(directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    directory = Path(directory)
    run = json.loads((directory / "run.json").read_text())
    rows = read_csv(directory / "samples.csv")
    commands = read_csv(directory / "commands.csv")
    plots = directory / "plots"
    plots.mkdir(exist_ok=True)
    title_prefix = "SIMULATED DATA — " if run["simulated"] else ""
    summaries = []
    for trial in run["trials"]:
        data = [row for row in rows if int(row["trial"]) == trial["id"]]
        sent = [row for row in commands if int(row["trial"]) == trial["id"]]
        summary = summarize(trial, data, sent, run["settings"])
        summaries.append(summary)
        if not data:
            continue
        t = column(data, "elapsed_s")
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, layout="constrained")
        axes[0].plot(t, column(data, "ideal_opening"), color="0.55", ls=":", label="Requested opening (ideal sine)")
        axes[0].plot(t, column(data, "expected_encoder_position"), color="#2463aa", label="Expected encoder (compensated)")
        axes[0].plot(t, column(data, "position"), color="#e78325", label="Measured encoder")
        if sent:
            # Transform each actually issued command into the same encoder coordinate.
            from sine_sweep import encoder_target
            expected = [encoder_target(float(row["opening"]), run["device"]["motor_travel_rad"],
                                       run["settings"]["close_compensation_rad"]) for row in sent]
            axes[0].step(column(sent, "elapsed_s"), expected, where="post", color="#2463aa", alpha=.25,
                         linewidth=.8, label="Issued command (compensated)")
        axes[0].set_ylabel("Opening [0..1]")
        axes[0].legend(loc="upper right", fontsize=8, ncol=2)
        axes[1].plot(t, 100 * (column(data, "expected_encoder_position") - column(data, "position")), color="#2463aa")
        axes[1].axhline(0, color="0.6", lw=.6)
        axes[1].set_ylabel("Error [% travel]")
        axes[2].plot(t, column(data, "torque_nm"), label="Motor torque estimate", color="#26856a")
        cap = trial["torque_limit_nm"]
        for sign in (-1, 1):
            axes[2].axhline(sign * cap, color="#cc5555", ls="--", label="Requested budget" if sign == 1 else None)
        axes[2].plot(t, column(data, "effective_torque_limit_nm"), color="0.5", ls=":", label="Firmware envelope (+)")
        axes[2].set_ylabel("Torque [Nm]")
        axes[2].legend(loc="upper right", fontsize=8, ncol=3)
        axes[3].plot(t, column(data, "velocity_rad_s"), color="#7555a1", label="Motor velocity")
        axes[3].set_ylabel("Velocity [rad/s]")
        temperature_axis = axes[3].twinx()
        temperature_axis.plot(t, column(data, "temperature_c"), color="#cb5a4a", alpha=.7)
        temperature_axis.set_ylabel("Temperature [C]", color="#cb5a4a")
        axes[3].set_xlabel("Host observation time since trial start [s]")
        warmup_end = run["settings"]["warmup_cycles"] / trial["frequency_hz"]
        for axis in axes:
            axis.axvspan(0, warmup_end, color="0.5", alpha=.1)
            axis.grid(alpha=.18)
        flags = " | reference speed LIMITED" if trial["reference_speed_limited"] else ""
        fig.suptitle(f"{title_prefix}Trial {trial['id']:02d} | {cap:g} Nm | {trial['frequency_hz']:g} Hz | "
                     f"{trial['speed_rad_s']:g} rad/s | {trial['status']}{flags}\n"
                     "Shaded = warmup excluded; encoder feedback does not measure jaw backlash")
        fig.savefig(plots / f"trial-{trial['id']:03d}.png", dpi=150)
        plt.close(fig)

    keys = list(dict.fromkeys(key for summary in summaries for key in summary))
    with (directory / "summary.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=keys)
        writer.writeheader()
        writer.writerows(summaries)

    fig, axes = plt.subplots(2, 3, figsize=(15, 8), layout="constrained")
    metrics = [("rmse_opening", "Tracking RMSE [% travel]", 100),
               ("amplitude_ratio", "Fundamental amplitude ratio", 1),
               ("apparent_phase_lag_ms", "Apparent phase lag [ms]", 1),
               ("torque_rms_nm", "Motor torque RMS [Nm]", 1),
               ("temperature_max_c", "Maximum temperature [C]", 1),
               ("observed_sample_hz", "Observed fresh samples [Hz]", 1)]
    groups = sorted({(s["torque_limit_nm"], s["speed_rad_s"]) for s in summaries})
    for (torque, speed) in groups:
        group = [s for s in summaries if s["torque_limit_nm"] == torque and s["speed_rad_s"] == speed]
        for axis, (metric, label, scale) in zip(axes.flat, metrics):
            valid = [s for s in group if s.get(metric) is not None and s["status"] == "complete"]
            frequencies = sorted({s["frequency_hz"] for s in valid})
            means = [np.mean([s[metric] * scale for s in valid if s["frequency_hz"] == f]) for f in frequencies]
            line, = axis.plot(frequencies, means, ".-", label=f"{torque:g} Nm / {speed:g} rad/s")
            for s in valid:
                axis.scatter(s["frequency_hz"], s[metric] * scale, color=line.get_color(), s=28,
                             marker="x" if s["reference_speed_limited"] else "o", alpha=.65)
            axis.set_xlabel("Sine frequency [Hz]")
            axis.set_ylabel(label)
            axis.grid(alpha=.2)
    axes[0, 1].axhline(1, color="0.5", ls=":", lw=1)
    axes[1, 2].axhline(run["settings"]["hz"], color="0.5", ls=":", lw=1)
    # Keep sub-millihertz timestamp noise from looking like a large rate change.
    rates = [s["observed_sample_hz"] for s in summaries if s.get("observed_sample_hz") is not None]
    axes[1, 2].set_ylim(min(rates + [run["settings"]["hz"]]) - 1,
                        max(rates + [run["settings"]["hz"]]) + 1)
    axes[1, 2].ticklabel_format(axis="y", style="plain", useOffset=False)
    if groups:
        axes[0, 0].legend(fontsize=8)
    fig.suptitle(f"{title_prefix}Sine tracking sweep | {run['status']}\n"
                 "Lines = repeat means; points = trials; x = reference speed below demand; unreliable phase omitted")
    fig.savefig(plots / "comparison.png", dpi=160)
    fig.savefig(plots / "comparison.svg")
    plt.close(fig)
    print(f"Plots: {plots.resolve()}\nMetrics: {(directory / 'summary.csv').resolve()}")
    return summaries


def generate_demo(directory):
    """Offline plotting fixture, NOT a motor model or measured performance."""
    from sine_sweep import (
        COMMAND_FIELDS,
        SAMPLE_FIELDS,
        encoder_target,
        make_trials,
        opening_at,
        parse_args,
        write_metadata,
    )

    args = parse_args(["--cycles", "3", "--warmup-cycles", "1", "--torques", ".2,.35,.5",
                       "--frequencies", ".1,.5,1", "--speeds", ".5"])
    directory.mkdir(parents=True, exist_ok=False)
    settings = vars(args).copy()
    settings["output"] = str(directory)
    travel = .5
    trials = make_trials(args, travel)
    with (directory / "samples.csv").open("w", newline="") as samples_file, \
            (directory / "commands.csv").open("w", newline="") as commands_file:
        samples = csv.DictWriter(samples_file, fieldnames=SAMPLE_FIELDS)
        commands = csv.DictWriter(commands_file, fieldnames=COMMAND_FIELDS)
        samples.writeheader()
        commands.writeheader()
        for trial in trials:
            frequency, cap = trial["frequency_hz"], trial["torque_limit_nm"]
            # Deliberately simple invented response to exercise plotting/phase extraction.
            tau = .04 + .035 / cap
            gain = 1 / math.sqrt(1 + (2 * math.pi * frequency * tau) ** 2)
            lag = math.atan(2 * math.pi * frequency * tau)
            duration = (args.cycles + args.warmup_cycles) / frequency
            for seq, t in enumerate(np.arange(0, duration, 1 / args.hz), 1):
                desired = opening_at(t, frequency, args.center, args.amplitude)
                position = encoder_target(args.center, travel, args.close_compensation_rad) + \
                    args.amplitude * (1 + args.close_compensation_rad / travel) * gain * math.cos(2 * math.pi * frequency * t - lag)
                samples.writerow({
                    "trial": trial["id"], "host_monotonic_s": t, "elapsed_s": t, "measured": int(t >= args.warmup_cycles / frequency),
                    "seq": seq, "command_id": seq, "ideal_opening": desired, "last_command_opening": desired,
                    "expected_encoder_position": encoder_target(desired, travel, args.close_compensation_rad), "position": position,
                    "velocity_rad_s": -args.amplitude * (travel + args.close_compensation_rad) * gain *
                    2 * math.pi * frequency * math.sin(2 * math.pi * frequency * t - lag),
                    "torque_nm": -cap * .7 * math.sin(2 * math.pi * frequency * t),
                    "temperature_c": 30 + cap * t * .02, "age_ms": 3., "effective_torque_limit_nm": 1.5,
                    "target_limited": False, "ready": True, "fault": "",
                })
                commands.writerow({"trial": trial["id"], "tick": seq - 1, "host_monotonic_s": t,
                                       "elapsed_s": t, "opening": desired, "api_call_ms": .05, "skipped_ticks": 0})
            trial.update(status="complete", native_feedback_hz=args.hz, native_command_hz=args.hz,
                         api_command_hz=args.command_hz, skipped_command_ticks=0)
    write_metadata(directory / "run.json", {"schema_version": 1, "simulated": True, "status": "complete", "settings": settings,
                                               "device": {"motor_travel_rad": travel}, "trials": trials})


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("directory", nargs="?", type=Path)
    parser.add_argument("--demo", type=Path, help="Generate SYNTHETIC data in a new directory without importing SDK")
    args = parser.parse_args()
    if (args.directory is None) == (args.demo is None):
        parser.error("provide either a capture directory or --demo NEW_DIRECTORY")
    if args.demo:
        generate_demo(args.demo)
    plot_run(args.demo or args.directory)


if __name__ == "__main__":
    main()
