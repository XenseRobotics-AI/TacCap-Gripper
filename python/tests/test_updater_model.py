"""Model initialization guards, with no serial or native SDK access."""

import importlib.util
import sys
import unittest
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch


class ModelTests(unittest.TestCase):
    def setUp(self):
        self.admin = Mock()
        flow = SimpleNamespace(_prepare_motor_admin=self.admin)
        sdk = SimpleNamespace(
            **{
                name: Mock()
                for name in (
                    "FollowerGripper",
                    "LeaderGripper",
                    "MotorOtaSession",
                    "MotorProtocol",
                    "OtaTargetVersion",
                    "scan_grippers",
                )
            }
        )
        path = Path(__file__).resolve().parents[2] / "apps/firmware_updater/device.py"
        spec = importlib.util.spec_from_file_location("model_test_driver", path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(
            sys.modules,
            {"full_ota_update": flow, "motor_ota_update": Mock(), "xense.taccap": sdk},
        ):
            spec.loader.exec_module(module)
        self.module = module
        self.device = module.Device(("test-s", "usb"), Mock())
        self.motor = Mock()
        self.motor.get_model.return_value = SimpleNamespace(from_flash=False)
        self.gripper = SimpleNamespace(motor=self.motor)

        @contextmanager
        def opened():
            yield self.gripper

        self.device.opened = opened
        self.device.power_cycle = Mock()
        self.device.inspect = Mock(return_value={"recorded": True, "model": "RS00"})

    def test_initialization_requires_power_cycle_and_readback(self):
        self.device.record_model("RS00")
        self.admin.assert_called_once_with(self.gripper)
        self.motor.set_model.assert_called_once_with(1)
        self.device.power_cycle.assert_called_once()
        self.device.inspect.assert_called_once()

    def test_already_recorded_model_is_not_overwritten(self):
        self.motor.get_model.return_value.from_flash = True
        with self.assertRaises(RuntimeError):
            self.device.record_model("RS00")
        self.motor.set_model.assert_not_called()

    def test_unknown_model_is_rejected_without_write(self):
        with self.assertRaises(ValueError):
            self.device.record_model("OTHER")
        self.motor.set_model.assert_not_called()

    def test_mismatched_readback_fails(self):
        self.device.inspect.return_value = {"recorded": True, "model": "EL05"}
        with self.assertRaises(RuntimeError):
            self.device.record_model("RS00")

    def test_progress_reports_real_counts_and_final_value(self):
        self.device.begin_transfer("电机传输", "包")
        self.device.progress(256, 1024)
        self.device.progress(1024, 1024)
        events = self.device.emit.call_args_list
        payloads = [call.args[1] for call in events if call.args[0] == "progress"]
        self.assertEqual(payloads[-1], {"done": 1024, "total": 1024, "unit": "包"})
        self.assertTrue(
            any(call.args[0] == "log" and "100%" in call.args[1] for call in events)
        )

    def test_young_clock_does_not_wait_for_five_seconds(self):
        flow = self.module.flow
        flow._read_uptime = Mock(side_effect=[200, 50])
        flow._scan_for = Mock(return_value="endpoint")
        flow._RestartDetector = Mock()
        flow._RestartDetector.return_value.observe.return_value = True
        self.device.endpoint = Mock(return_value="endpoint")
        with patch.object(self.module.time, "sleep") as sleep:
            type(self.device).power_cycle(self.device, "test")
        sleep.assert_not_called()
        flow._RestartDetector.assert_called_once_with(200)
        self.assertEqual(flow._read_uptime.call_count, 2)
        self.device.emit.assert_any_call("power_done", "")

    def test_missing_usb_is_reported_and_never_passes_as_restart(self):
        flow = self.module.flow
        flow._read_uptime = Mock(return_value=200)
        flow._scan_for = Mock(return_value=None)
        flow._RestartDetector = Mock()
        self.device.endpoint = Mock(return_value="endpoint")
        with (
            patch.object(self.module.time, "monotonic", side_effect=[0, 1, 4, 4, 181]),
            patch.object(self.module.time, "sleep"),
            self.assertRaisesRegex(RuntimeError, "未发现目标 USB"),
        ):
            type(self.device).power_cycle(self.device, "test")
        flow._RestartDetector.return_value.observe.assert_not_called()
        self.assertTrue(
            any(c.args[0] == "power_status" for c in self.device.emit.call_args_list)
        )
        self.device.emit.assert_any_call("power_done", "")

    def test_failed_cycle_never_reports_initialized(self):
        self.device.power_cycle.side_effect = RuntimeError("timeout")
        with self.assertRaises(RuntimeError):
            self.device.record_model("RS00")
        self.device.inspect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
