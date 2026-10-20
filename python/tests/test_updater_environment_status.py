"""Readiness never treats SDK/OS success as a successful device handshake."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "apps/firmware_updater"))
from environment_status import assess  # noqa: E402


def environment(*serials):
    return dict(
        runtime={"ok": True},
        architecture="x86_64",
        kernel="test",
        user="tester",
        dialout_active=True,
        ready_for_probe=bool(serials),
        scope="Fixture only; no serial ports opened.",
        bridges=[{"serial": s} for s in serials],
        ports=[
            dict(
                serial=s,
                node_present=True,
                read_write=True,
                owners=[],
                driver="cdc_acm",
                node="/dev/ttyACM0",
                stable_present=True,
            )
            for s in serials
        ],
        issues=[],
        blocking=False,
    )


def test_no_usb_never_indicates_ready_even_when_sdk_loads():
    state = assess(environment())
    assert state["tone"] == "warning" and not state["system_ready"]
    assert "不能开始更新" in state["title"]


def test_os_pass_still_requires_identity_readback():
    state = assess(environment("usb"))
    assert state["system_ready"] and not state["connection_verified"]
    assert state["tone"] == "warning" and "尚需连接验证" in state["title"]


def test_verified_identity_and_recorded_model_allow_preflight_not_success_claim():
    state = assess(environment("usb"), ("TESTs", "usb"), {"recorded": True})
    assert state["tone"] == "success" and state["configured"]
    assert "预检" in state["title"] and "仍会检查" in state["next_step"]


@pytest.mark.parametrize(
    "failure", ["runtime", "permission", "owner", "missing_node", "communication"]
)
def test_failures_block_ready_and_use_red(failure):
    report = environment("usb")
    if failure == "runtime":
        report["runtime"]["ok"] = False
    elif failure == "permission":
        report["ports"][0]["read_write"] = False
    elif failure == "owner":
        report["ports"][0]["owners"] = [{"pid": 123}]
    elif failure == "missing_node":
        report["ports"][0]["node_present"] = False
    else:
        report["connection_error"] = "GetSn timeout"
    state = assess(report, ("TESTs", "usb"), {"recorded": True})
    assert state["tone"] == "error" and not state["configured"]


def test_healthy_other_port_does_not_validate_selected_device():
    state = assess(
        environment("other-usb"), ("TESTs", "missing-usb"), {"recorded": True}
    )
    assert not state["system_ready"] and not state["connection_verified"]


def test_unrecorded_model_and_unknown_driver_are_not_green():
    report = environment("usb")
    assert assess(report, ("TESTs", "usb"), {"recorded": False})["tone"] == "warning"
    report["ports"][0]["driver"] = "unknown"
    assert assess(report, ("TESTs", "usb"), {"recorded": True})["tone"] == "warning"


def test_assessment_is_localized_without_unicode_fallback():
    import i18n

    old = i18n.language()
    try:
        i18n.set_language("en")
        for report in (environment(), environment("usb")):
            for info in (None, {"recorded": True}, {"recorded": False}):
                state = assess(report, ("TESTs", "usb"), info)
                values = [state["title"], state["badge"], state["next_step"]]
                values += [v for c in state["checks"] for v in (c["name"], c["value"])]
                for value in values:
                    translated = i18n.tr(value)
                    assert not i18n.CJK.search(translated) and "\\u" not in translated
    finally:
        i18n.set_language(old)
