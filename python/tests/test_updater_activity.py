"""Progress is based on transfer callbacks, never elapsed-time guesses."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "apps/firmware_updater"))
from activity import Activity


class ActivityTests(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.state = Activity(lambda: self.now)
        self.state.begin("等待协议")

    def test_wait_has_elapsed_but_no_fake_percent(self):
        self.now = 65
        self.assertIn("01:05", self.state.detail())
        self.assertNotIn("%", self.state.detail())

    def test_transfer_percentage_and_units(self):
        self.state.update(256, 1024, "包")
        self.assertIn("25%", self.state.detail())
        self.assertIn("256/1,024 包", self.state.detail())

    def test_transfer_complete_does_not_mean_update_complete(self):
        self.state.update(100, 100, "B")
        self.assertIn("仍需完成校验", self.state.detail())
        self.assertTrue(self.state.active)

    def test_stalled_transfer_not_faked_forward(self):
        self.state.update(20, 100, "B")
        self.now = 10
        self.assertIn("20%", self.state.detail())
        self.assertIn("等待设备回执", self.state.detail())

    def test_phase_change_clears_old_percentage(self):
        self.state.update(100, 100, "B")
        self.now = 8
        self.state.set_stage("等待断电")
        self.assertNotIn("%", self.state.detail())
        self.assertIn("00:08", self.state.detail())

    def test_heartbeat_throttles_and_stops(self):
        self.assertIsNone(self.state.heartbeat())
        self.now = 5
        self.assertIn("等待协议", self.state.heartbeat())
        self.assertIsNone(self.state.heartbeat())
        self.state.finish()
        self.now = 10
        self.assertIsNone(self.state.heartbeat())


if __name__ == "__main__":
    unittest.main()
