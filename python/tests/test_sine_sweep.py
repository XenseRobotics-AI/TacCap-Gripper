"""Offline checks for measurement semantics and the timed hardware example."""
import csv
import importlib
import io
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


@pytest.fixture
def sweep(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / "examples"))
    return importlib.import_module("sine_sweep")


@pytest.mark.parametrize("argv", [
    ["--torques", "0"], ["--torques", "nan"], ["--torques", "0.2,0.2"],
    ["--torques", "1.3"], ["--torques", "bad"], ["--speeds", "51"],
    ["--center", "0.1", "--amplitude", "0.2"], ["--amplitude", "0"],
    ["--frequencies", "6"], ["--command-hz", "0"], ["--hz", "30"],
    ["--cycles", "1"], ["--repeats", "0"], ["--rest", "-1"],
    ["--close-compensation-rad", "0.06"], ["--settle-timeout", "nan"],
    ["--max-temperature", "100"], ["--warmup-cycles", "-1"],
])
def test_invalid_experiment_rejected_before_connecting(sweep, argv):
    with pytest.raises(SystemExit):
        sweep.parse_args(argv)


def test_plan_accounts_for_compensation_speed_and_repeats(sweep):
    args = sweep.parse_args(["--repeats", "2"])
    trials = sweep.make_trials(args, travel=.5)
    assert len(trials) == 40
    assert trials == sweep.make_trials(args, travel=.5)
    fast = next(t for t in trials if t["frequency_hz"] == .5)
    assert fast["required_peak_speed_rad_s"] == pytest.approx(2 * math.pi * .5 * .25 * .53)
    assert not fast["reference_speed_limited"]
    assert sweep.encoder_target(0, .5, .03) == 0
    assert sweep.encoder_target(1, .5, .03) == 1
    assert sweep.encoder_target(.5, .5, .03) == pytest.approx(.47)


def test_motion_sweep_covers_higher_budgets_without_baseline_slew_bottleneck(sweep):
    args = sweep.parse_args([])
    # Actual travel from the 2026-09-11 empty-baseline capture, not the demo's 0.5 rad.
    travel = 1.2259337715804577
    trials = sweep.make_trials(args, travel)
    assert args.torques == [.35, .6, .9, 1.2]
    assert args.frequencies == [.25, .5, 1, 2, 3]
    assert args.cycles / max(args.frequencies) >= 2.5
    assert args.warmup_cycles / max(args.frequencies) >= .5
    assert not any(t["reference_speed_limited"] for t in trials)
    assert max(t["required_peak_speed_rad_s"] for t in trials) == pytest.approx(5.9184484653)
    args.speeds = [.5]
    limited = [t for t in sweep.make_trials(args, travel) if t["reference_speed_limited"]]
    assert {t["frequency_hz"] for t in limited} == {.5, 1, 2, 3}
    assert min(t["required_peak_speed_rad_s"] for t in limited) == pytest.approx(.9864080775)


def test_higher_frequency_respects_command_and_feedback_sampling(sweep):
    # 5 Hz is allowed at 100 Hz, but neither a slower input nor slower feedback
    # may silently reduce the configured samples per period below 20.
    sweep.parse_args(["--frequencies", "5", "--command-hz", "100", "--hz", "100"])
    for rates in (["--command-hz", "50"], ["--hz", "50"]):
        with pytest.raises(SystemExit):
            sweep.parse_args(["--frequencies", "3", *rates])


class Clock:
    def __init__(self):
        self.now = 1000.

    def __call__(self):
        return self.now

    def sleep(self, duration):
        self.now += max(duration, 1e-9)


@dataclass
class Timing:
    feedback_frames: int
    command_frames: int


@dataclass
class Info:
    motor_travel_rad: float = .5
    firmware_version: tuple = (1, 1, 8)


class FakeGripper:
    def __init__(self, clock, *, fault=False):
        self.clock = clock
        self.started = clock()
        self.info = Info()
        self.calls = []
        self.fault = fault
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.closed = True

    def start(self):
        pass

    @property
    def state(self):
        elapsed = self.clock() - self.started
        # 50 Hz feedback, polled faster; duplicate reads must not duplicate CSV rows.
        seq = int(elapsed * 50)
        return SimpleNamespace(position=.5, velocity_rad_s=0., torque_nm=.1, temperature_c=30.,
                               age_ms=2., seq=seq, command_id=len(self.calls), target_error_rad=0.,
                               effective_torque_limit_nm=1., target_limited=False, ready=True,
                               fault="test fault" if self.fault and elapsed > .3 else None)

    @property
    def timing(self):
        return Timing(self.state.seq, len(self.calls))

    def move(self, position, **limits):
        self.calls.append((self.clock(), position, limits))
        # A deliberately long first API call exercises skipped ticks, not a burst queue.
        self.clock.sleep(.055 if len(self.calls) == 1 else .0002)


def capture(sweep, *, fault=False):
    args = sweep.parse_args(["--frequencies", "1", "--cycles", "2", "--warmup-cycles", "0"])
    trial = sweep.make_trials(args, .5)[0]
    clock = Clock()
    gripper = FakeGripper(clock, fault=fault)
    samples, commands = io.StringIO(), io.StringIO()
    sw, cw = csv.writer(samples), csv.writer(commands)
    sw.writerow(sweep.SAMPLE_FIELDS)
    cw.writerow(sweep.COMMAND_FIELDS)
    if fault:
        with pytest.raises(RuntimeError, match="test fault"):
            sweep.record_trial(gripper, args, trial, sw, cw, clock=clock, sleep=clock.sleep)
    else:
        sweep.record_trial(gripper, args, trial, sw, cw, clock=clock, sleep=clock.sleep)
    return trial, gripper, list(csv.DictReader(io.StringIO(samples.getvalue()))), list(csv.DictReader(io.StringIO(commands.getvalue())))


def test_timed_loop_skips_late_ticks_and_records_only_new_sequences(sweep):
    trial, gripper, samples, commands = capture(sweep)
    assert trial["status"] == "complete"
    ticks = [int(row["tick"]) for row in commands]
    assert ticks[:2] == [0, 5]
    assert len(ticks) == len(set(ticks))
    assert trial["skipped_command_ticks"] >= 4
    assert len(samples) == len({row["seq"] for row in samples})
    assert 95 <= len(samples) <= 100
    assert all(call[2]["wait"] is False for call in gripper.calls)
    assert all(call[2]["torque_limit_nm"] == trial["torque_limit_nm"] for call in gripper.calls)
    assert min(np.diff([float(row["elapsed_s"]) for row in commands])) > .001


def test_fault_keeps_partial_data_and_stops_the_trial(sweep):
    trial, _, samples, _ = capture(sweep, fault=True)
    assert trial["status"] == "aborted"
    assert trial["duration_s"] < .4
    assert samples[-1]["fault"] == "test fault"


def test_existing_capture_is_not_overwritten(sweep, tmp_path):
    with pytest.raises(FileExistsError):
        sweep.main(["--output", str(tmp_path)])


@pytest.mark.parametrize("interrupt", [False, True])
def test_main_closes_context_and_saves_abort_metadata(sweep, tmp_path, monkeypatch, interrupt):
    clock = Clock()
    gripper = FakeGripper(clock, fault=True)
    if interrupt:
        def move(*args, **kwargs):
            raise KeyboardInterrupt
        gripper.move = move
    monkeypatch.setitem(sys.modules, "xense.taccap", SimpleNamespace(
        Gripper=lambda *args, **kwargs: gripper, __version__="test"))
    monkeypatch.setattr("builtins.input", lambda prompt: "")
    monkeypatch.setattr(sweep, "prepare", lambda *args: None)
    record = sweep.record_trial
    monkeypatch.setattr(sweep, "record_trial", lambda *args: record(*args, clock=clock, sleep=clock.sleep))
    output = tmp_path / "capture"
    with pytest.raises(KeyboardInterrupt if interrupt else RuntimeError):
        sweep.main(["--output", str(output), "--no-plot", "--torques", ".35", "--frequencies", "1"])
    assert gripper.closed
    metadata = json.loads((output / "run.json").read_text())
    assert metadata["status"] == metadata["trials"][0]["status"] == "aborted"
    assert (output / "samples.csv").is_file()
    assert (output / "commands.csv").is_file()


@pytest.mark.parametrize("changes, message", [
    ({"temperature_c": 60}, "Temperature"), ({"age_ms": 201}, "Stale"),
    ({"velocity_rad_s": float("nan")}, "Non-finite"),
])
def test_feedback_abort_conditions(sweep, changes, message):
    state = FakeGripper(Clock()).state
    state.__dict__.update(changes)
    with pytest.raises(RuntimeError, match=message):
        sweep.check_state(state, 60)


def test_fit_recovers_known_gain_and_delay_with_irregular_sampling(sweep):
    plot = importlib.import_module("plot_sine_sweep")
    rng = np.random.default_rng(10)
    t = np.sort(rng.uniform(0, 12, 1000))
    frequency, amplitude, lag = .5, .2, .09
    values = .4 + amplitude * np.cos(2 * np.pi * frequency * (t - lag))
    fitted_amp, phase, r2 = plot.fit_sine(t, values, frequency, plot.time_weights(t))
    assert fitted_amp == pytest.approx(amplitude)
    assert phase / (2 * np.pi * frequency) == pytest.approx(lag)
    assert r2 == pytest.approx(1)


def test_metrics_exclude_warmup_and_suppress_false_stationary_phase(sweep, tmp_path):
    plot = importlib.import_module("plot_sine_sweep")
    data = tmp_path / "demo"
    plot.generate_demo(data)
    run = json.loads((data / "run.json").read_text())
    trial = run["trials"][0]
    rows = [row for row in plot.read_csv(data / "samples.csv") if int(row["trial"]) == trial["id"]]
    summary = plot.summarize(trial, rows, [], run["settings"])
    tau = .04 + .035 / trial["torque_limit_nm"]
    omega = 2 * math.pi * trial["frequency_hz"]
    assert summary["amplitude_ratio"] == pytest.approx(1 / math.sqrt(1 + (omega * tau) ** 2))
    assert summary["apparent_phase_lag_ms"] == pytest.approx(math.atan(omega * tau) / omega * 1000)
    for row in rows:
        if row["measured"] == "0":
            row["position"] = "999"
    assert plot.summarize(trial, rows, [], run["settings"])["rmse_opening"] == summary["rmse_opening"]
    for row in rows:
        row["position"] = ".5"
    stationary = plot.summarize(trial, rows, [], run["settings"])
    assert not stationary["fit_reliable"]
    assert stationary["apparent_phase_lag_ms"] is None
    trial["status"] = "aborted"
    assert not plot.summarize(trial, rows, [], run["settings"])["fit_reliable"]
