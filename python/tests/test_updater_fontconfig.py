"""Fontconfig initialization must not depend on a builder's conda prefix."""

import importlib
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

APP_DIR = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
sys.path.insert(0, str(APP_DIR))
fontconfig_runtime = importlib.import_module("fontconfig_runtime")


class FontconfigRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name) / "fonts.conf"
        self.config.touch()
        for context in (
            patch.dict(os.environ, {}, clear=True),
            patch.object(sys, "platform", "linux"),
            patch.object(fontconfig_runtime, "SYSTEM_CONFIG", self.config),
        ):
            context.start()
            self.addCleanup(context.stop)

    def test_system_configuration_selected_and_idempotent(self):
        self.assertTrue(fontconfig_runtime.configure_fontconfig())
        self.assertEqual(os.environ["FONTCONFIG_FILE"], str(self.config))
        self.assertEqual(os.environ["FONTCONFIG_PATH"], str(self.config.parent))
        self.assertFalse(fontconfig_runtime.configure_fontconfig())

    def test_empty_overrides_use_system_configuration(self):
        os.environ.update(FONTCONFIG_FILE="", FONTCONFIG_PATH="")
        self.assertTrue(fontconfig_runtime.configure_fontconfig())
        self.assertEqual(os.environ["FONTCONFIG_FILE"], str(self.config))

    def test_explicit_configuration_preserved(self):
        for key in ("FONTCONFIG_FILE", "FONTCONFIG_PATH"):
            with (
                self.subTest(key=key),
                patch.dict(os.environ, {key: "/custom/fonts"}, clear=True),
            ):
                self.assertFalse(fontconfig_runtime.configure_fontconfig())
                self.assertEqual(dict(os.environ), {key: "/custom/fonts"})

    def test_missing_system_configuration_does_not_hide_error(self):
        self.config.unlink()
        self.assertFalse(fontconfig_runtime.configure_fontconfig())
        self.assertEqual(dict(os.environ), {})

    def test_non_linux_environment_untouched(self):
        with patch.object(sys, "platform", "darwin"):
            self.assertFalse(fontconfig_runtime.configure_fontconfig())
        self.assertEqual(dict(os.environ), {})


if __name__ == "__main__":
    unittest.main()
