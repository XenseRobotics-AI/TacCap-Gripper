"""Bundled firmware policy tests, no SDK or hardware required."""

import hashlib
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "apps/firmware_updater"))
import bundled


class BundledTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.catalog = {"schema": 1, "images": []}
        for (kind, model), relative in bundled.FILES.items():
            data = f"test-only/{kind}/{model}".encode()
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
            self.catalog["images"].append(
                dict(
                    kind=kind,
                    model=model,
                    version="1.2.3",
                    sha256=hashlib.sha256(data).hexdigest(),
                    warning="",
                )
            )

    def test_all_three_defaults_are_verified(self):
        versions = bundled.verify_all(self.catalog, self.root)
        self.assertEqual(
            set(versions), {"mcu/slave", "mcu/master", "motor/EL05", "motor/RS00"}
        )

    def test_defaults_select_highest_approved_version(self):
        catalog = {
            "images": [
                dict(kind="motor", model="EL05", version="1.0.5.0.2", file="old.bin"),
                dict(kind="motor", model="EL05", version="1.0.5.0.4", file="new.bin"),
            ]
        }
        self.assertEqual(bundled.approved_defaults(catalog)["motor", "EL05"], "new.bin")

    def test_unknown_motor_never_defaults_to_el05(self):
        for model in (None, "", "UNKNOWN"):
            self.assertIsNone(
                bundled.default_image("motor", model, self.catalog, self.root)
            )

    def test_tampered_image_rejected(self):
        (self.root / bundled.FILES["motor", "RS00"]).write_bytes(b"changed")
        with self.assertRaises(ValueError):
            bundled.verify_all(self.catalog, self.root)

    def test_missing_image_rejected(self):
        (self.root / bundled.FILES["motor", "EL05"]).unlink()
        with self.assertRaises(FileNotFoundError):
            bundled.verify_all(self.catalog, self.root)


if __name__ == "__main__":
    unittest.main()
