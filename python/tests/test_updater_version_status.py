"""Version colors retain provenance and never recommend accidental downgrades."""

import sys
from pathlib import Path

import pytest

APP = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
sys.path.insert(0, str(APP))
import core  # noqa: E402
from version_status import device_summary, version_state  # noqa: E402


@pytest.mark.parametrize(
    "value,target,state",
    [
        ("1.2.16", "1.2.16", "current"),
        ("1.2.16.0", "1.2.16", "current"),
        ("1.2.9", "1.2.16", "outdated"),
        ("1.2.17", "1.2.16", "newer"),
        ("未知", "1.2.16", "unknown"),
        ("1.0.5.0.4（Flash 存档，非实时）", "1.0.5.0.4", "current"),
        ("1.0.5.0.4", None, "unknown"),
    ],
)
def test_version_states(value, target, state):
    assert version_state(value, target)[0] == state


def test_each_component_gets_its_own_color():
    info = dict(
        model="EL05",
        recorded=True,
        mcu="1.2.14",
        motor="1.0.5.0.4（Flash 存档，非实时）",
    )
    html, plain, tone, status = device_summary(
        info, "slave", core.catalog_at(APP / "catalog.json")
    )
    assert "#B42318" in html and "#146448" in html
    assert "请更新至 1.2.16" in plain
    assert "Flash 存档，非实时" in html
    assert tone == "error" and "建议" in status


def test_master_does_not_report_motor_and_unrecorded_is_unknown():
    catalog = core.catalog_at(APP / "catalog.json")
    info = dict(model="EL05", recorded=False, mcu="1.2.6", motor="1.0.5.0.4")
    html, plain, tone, status = device_summary(info, "master", catalog)
    assert "电机" not in html and tone == "success"
    html, plain, tone, status = device_summary(info, "slave", catalog)
    assert "电机 1.0.5.0.4 · 版本待核实" in plain


def test_rendering_escapes_device_text():
    info = dict(model="<b>unknown</b>", recorded=False, mcu="未知", motor="<bad>")
    html, _, tone, _ = device_summary(
        info, "slave", core.catalog_at(APP / "catalog.json")
    )
    assert "<bad>" not in html and "&lt;bad&gt;" in html
    assert tone == "warning"
