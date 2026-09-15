"""Offline measurement and cleanup regressions; no hardware is opened."""
import importlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def benchmark(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / "examples"))
    return importlib.import_module("frequency_benchmark")


@pytest.mark.parametrize("argv", [
    ["--sdk-hz", "30"], ["--command-hz", "0"], ["--read-hz", "2001"],
    ["--sdk-hz", "50,50"], ["--duration", "nan"], ["--duration", "0"],
    ["--command-hz", "501"], ["--torque", "1.3"], ["--warmup", "-1"],
    ["--motion", "sine", "--sine-hz", "2"], ["--amplitude", "0.7"],
])
def test_invalid_settings_do_not_connect(benchmark, argv):
    with pytest.raises(SystemExit):
        benchmark.parse_args(argv)


class Clock:
    now = 0.

    def __call__(self):
        return self.now

    def sleep(self, duration):
        self.now += max(1e-9, duration)


@dataclass
class Timing:
    feedback_frames: int
    command_frames: int


class FakeGripper:
    def __init__(self, clock, *, fault=False):
        self.clock = clock
        self.calls = []
        self.fault = fault

    def set_position(self, position):
        self.calls.append((self.clock(), position))
        self.clock.sleep(.055 if len(self.calls) == 1 else .0002)

    @property
    def state(self):
        return SimpleNamespace(seq=int(self.clock() * 50), position=.4, target=.4,
            velocity_rad_s=0., torque_nm=.1, temperature_c=30., age_ms=2.,
            target_error_rad=0., effective_torque_limit_nm=1.,
            fault="injected fault" if self.fault and self.clock() > .15 else None)

    @property
    def timing(self):
        # Native writes and feedback are deliberately independent of Python call counts.
        return Timing(int(self.clock() * 50), int(self.clock() * 40))


def capture(benchmark, read_hz=500, fault=False):
    clock = Clock()
    gripper = FakeGripper(clock, fault=fault)
    args = benchmark.parse_args(["--duration", "1"])
    trial = {"id": 1, "command_hz": 100, "read_hz": read_hz, "hold_position": .4}
    commands, reads = [], []
    return clock, gripper, args, trial, commands, reads


def test_rates_do_not_count_cached_reads_as_feedback(benchmark):
    clock, gripper, args, trial, commands, reads = capture(benchmark)
    benchmark.record_trial(gripper, args, trial, commands, reads, clock=clock, sleep=clock.sleep)
    metrics = trial["metrics"]
    assert trial["status"] == "complete"
    assert metrics["native_feedback_hz"] == pytest.approx(50, abs=1)
    assert metrics["native_command_hz"] == pytest.approx(40, abs=1)
    assert metrics["api_read_hz"] > 400
    assert metrics["observed_feedback_hz"] <= metrics["native_feedback_hz"]
    assert metrics["duplicate_reads"] > 300
    assert metrics["command_skipped_ticks"] >= 5
    assert metrics["command_call_ms"]["max"] == pytest.approx(55)
    assert all(b[0] - a[0] >= .009 for a, b in zip(gripper.calls, gripper.calls[1:]))


def test_slow_reads_record_unobserved_updates_not_packet_loss(benchmark):
    clock, gripper, args, trial, commands, reads = capture(benchmark, read_hz=10)
    benchmark.record_trial(gripper, args, trial, commands, reads, clock=clock, sleep=clock.sleep)
    metrics = trial["metrics"]
    assert metrics["unobserved_cache_updates"] > 20
    assert metrics["observed_feedback_hz"] <= 11
    assert metrics["native_feedback_hz"] == pytest.approx(50, abs=1)
    assert not any("loss" in key or "dropped" in key for key in metrics)


def test_fault_preserves_partial_counters(benchmark):
    clock, gripper, args, trial, commands, reads = capture(benchmark, fault=True)
    with pytest.raises(RuntimeError, match="injected fault"):
        benchmark.record_trial(gripper, args, trial, commands, reads, clock=clock, sleep=clock.sleep)
    assert trial["status"] == "aborted"
    assert 0 < trial["duration_s"] < 1
    assert trial["metrics"]["native_feedback_hz"] > 0
    assert commands and reads


def test_abort_closes_device_and_saves_run(benchmark, monkeypatch, tmp_path):
    class Session:
        closed = False
        info = Timing(0, 0)  # Any dataclass can stand in for immutable device info here.

        def __init__(self, *args, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            Session.closed = True

        def start(self):
            raise RuntimeError("start failure")

    monkeypatch.setitem(sys.modules, "xense.taccap", SimpleNamespace(Gripper=Session, __version__="test"))
    monkeypatch.setattr("builtins.input", lambda *args: "")
    output = tmp_path / "aborted"
    with pytest.raises(RuntimeError, match="start failure"):
        benchmark.main(["--output", str(output), "--no-plot"])
    data = json.loads((output / "run.json").read_text())
    assert Session.closed
    assert data["status"] == data["trials"][0]["status"] == "aborted"
    assert (output / "trial-001-reads.csv").exists()


def test_offline_plot_and_metrics(benchmark, tmp_path):
    pytest.importorskip("matplotlib")
    clock, gripper, args, trial, commands, reads = capture(benchmark)
    trial.update(sdk_hz=50)
    benchmark.record_trial(gripper, args, trial, commands, reads, clock=clock, sleep=clock.sleep)
    benchmark.save_trial(tmp_path, trial, commands, reads)
    (tmp_path / "run.json").write_text(json.dumps({"status": "SIMULATED TEST DATA", "simulated": True,
                                                "settings": {"motion": "hold"}, "trials": [trial]}))
    benchmark.main(["--plot", str(tmp_path)])
    assert (tmp_path / "frequency-summary.png").stat().st_size > 1000
    assert (tmp_path / "frequency-summary.svg").stat().st_size > 1000
    assert benchmark.distribution([]) is None
    assert benchmark.distribution([0, 100])["p95"] == 95
