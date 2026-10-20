"""No serial opens: model OS prerequisites using a temporary sysfs tree."""

import importlib.util
import sys
from pathlib import Path
from unittest.mock import patch

import pytest

APP = Path(__file__).resolve().parents[2] / "apps/firmware_updater"
sys.path.insert(0, str(APP))
import host_diagnostics as health  # noqa: E402


@pytest.fixture
def host(tmp_path):
    sys_root, dev = tmp_path / "sys", tmp_path / "dev"
    usb = sys_root / "bus/usb/devices/1-2"
    usb.mkdir(parents=True)
    for name, value in {
        "idVendor": "1a86",
        "product": "USB Dual Serial",
        "serial": "5C96090844",
    }.items():
        (usb / name).write_text(value)
    interface = usb / "1-2:1.2"
    interface.mkdir()
    (interface / "bInterfaceNumber").write_text("02")
    port = interface / "tty/ttyACM1"
    port.mkdir(parents=True)
    (sys_root / "class/tty").mkdir(parents=True)
    (sys_root / "class/tty/ttyACM1").symlink_to(port)
    dev.mkdir()
    (dev / "ttyACM1").touch()
    alias = dev / "serial/by-id/usb-1a86_USB_Dual_Serial_5C96090844-if02"
    alias.parent.mkdir(parents=True)
    alias.symlink_to(dev / "ttyACM1")
    return sys_root, dev, usb, interface, alias


def collect(host, **overrides):
    return health.collect(
        sys_root=host[0],
        dev_root=host[1],
        access=lambda p: True,
        char_node=lambda p: p.exists(),
        **overrides,
    )


def test_permission_failure_is_explicit_not_no_device(host):
    report = health.collect(
        sys_root=host[0],
        dev_root=host[1],
        access=lambda p: False,
        char_node=lambda p: True,
    )
    assert len(report["ports"]) == 1 and not report["ready_for_probe"]
    assert "permission_denied" in [i["code"] for i in report["issues"]]
    with pytest.raises(RuntimeError, match="读写权限"):
        health.assert_access(report=report)
    assert "MCU 已应答" not in health.summary(report)


def test_accessible_raw_tty_keeps_sdk_fallback_with_stable_warning(host):
    host[4].unlink()
    report = collect(host)
    assert report["ready_for_probe"]
    assert any(i["code"] == "stable_path_missing" for i in report["issues"])
    health.assert_access(report=report)
    assert "不代表 MCU 已应答" in health.summary(report)


def test_foreign_debug_interfaces_are_not_granted_or_probed(host):
    (host[3] / "bInterfaceNumber").write_text("00")
    report = collect(host)
    assert report["ports"] == []
    assert any(i["code"] == "tty_missing" for i in report["issues"])


def test_driver_node_missing_differentiates_usb_from_permission(host):
    (host[1] / "ttyACM1").unlink()
    report = collect(host)
    assert not report["ready_for_probe"]
    assert any(i["code"] == "node_missing" for i in report["issues"])
    assert not any(i["code"] == "permission_denied" for i in report["issues"])


def test_no_bridge_is_not_diagnosed_as_missing_sdk(tmp_path):
    report = health.collect(sys_root=tmp_path / "sys", dev_root=tmp_path / "dev")
    assert not report["ports"] and not report["bridges"]
    health.assert_access(report=report)
    assert "USB 线缆" in health.summary(report)


def test_identity_specific_permission_check_does_not_use_other_port(host):
    report = collect(host)
    blocked = dict(report["ports"][0], serial="BLOCKED", read_write=False)
    report["ports"].append(blocked)
    report["issues"].append({"code": "permission_denied", "message": "读写权限不足"})
    health.assert_access(chip_serial="5C96090844", report=report)
    with pytest.raises(RuntimeError, match="读写权限"):
        health.assert_access(chip_serial="BLOCKED", report=report)


def test_diagnostic_import_does_not_scan_or_send_commands(host):
    import xense.taccap as sdk

    with (
        patch.object(health, "collect", return_value=collect(host)),
        patch.object(sdk, "scan_grippers", side_effect=AssertionError("must not scan")),
    ):
        report = health.diagnostic_report()
    assert report["runtime"]["ok"] and report["runtime"]["native"]


def test_serial_rules_installer_is_targeted_and_non_world_writable(tmp_path):
    root = APP.parents[1]
    packaging = root / "packaging/firmware_updater"
    sys.path.insert(0, str(packaging))
    spec = importlib.util.spec_from_file_location(
        "serial_rule_packager", packaging / "build_deb.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    (tmp_path / "DEBIAN").mkdir()
    module.install_serial_rules(tmp_path)
    rules = (tmp_path / "usr/lib/udev/rules.d" / health.RULE_NAME).read_text()
    script = tmp_path / "DEBIAN/postinst"
    assert 'ATTRS{idVendor}!="1a86"' in rules
    assert 'ATTRS{product}!="USB Dual Serial"' in rules
    assert 'ENV{.TACCAP_IFNUM}!="02"' in rules
    assert 'MODE="0660"' in rules and 'TAG+="uaccess"' in rules
    assert 'ENV{ID_MM_PORT_IGNORE}="1"' in rules
    assert "SYMLINK+=" in rules and "666" not in rules and "777" not in rules
    assert script.stat().st_mode & 0o111
    assert "--reload-rules" in script.read_text()
    assert (
        "usermod" not in script.read_text()
        and "trigger" not in script.read_text().replace("broad device triggers", "")
    )


def guided(host, *, writable=True, runtime=True, probe=None, owners=None):
    import environment_guidance as guide

    report = health.collect(
        sys_root=host[0],
        dev_root=host[1],
        access=lambda _: writable,
        char_node=lambda _: True,
    )
    report["runtime"] = {"ok": runtime}
    guide.enrich(
        report, probe=probe or (lambda _: (0, "")), find_owners=lambda _: owners or []
    )
    return report


def test_permission_commands_are_manual_scoped_and_no_developer_sdk(host):
    from environment_guidance import commands

    report = guided(host, writable=False)
    report["dialout_active"] = False
    assert report["blocking"]
    text = commands(report)
    assert 'MODE="0660"' in text and "--reload-rules" in text
    assert "--trigger" not in text and "chmod 777" not in text
    assert "pip install" not in text and "conda install" not in text
    assert 'ENV{.TACCAP_IFNUM}!="02"' in text


def test_pending_group_membership_requires_full_login_not_usermod_again(host):
    import environment_guidance as guide

    report = health.collect(
        sys_root=host[0],
        dev_root=host[1],
        access=lambda _: False,
        char_node=lambda _: True,
    )
    report.update(dialout_configured=True, dialout_active=False, runtime={"ok": True})
    guide.enrich(report, probe=lambda _: (0, ""), find_owners=lambda _: [])
    assert "usermod" not in guide.commands(report)
    assert "仅开新终端不够" in health.summary(report)


@pytest.mark.parametrize("arch", ["x86_64", "aarch64"])
def test_runtime_failure_blocks_scan_on_both_platforms(host, arch):
    with patch.object(health.platform, "machine", return_value=arch):
        report = guided(host, runtime=False)
    assert report["architecture"] == arch and report["blocking"]
    assert any(i["code"] == "sdk_unavailable" for i in report["issues"])


def test_modemmanager_active_is_risk_not_proven_owner(host):
    report = guided(
        host, probe=lambda argv: (0, "active" if argv[0] == "systemctl" else "")
    )
    issue = next(i for i in report["issues"] if i["code"] == "modemmanager_risk")
    assert "不是已确认" in issue["message"]
    assert report["ports"][0]["owners"] == []
    assert not report["blocking"]


def test_modemmanager_ignored_port_does_not_prompt_service_stop(host):
    report = guided(
        host,
        probe=lambda argv: (
            0,
            "active" if argv[0] == "systemctl" else "ID_MM_PORT_IGNORE=1",
        ),
    )
    assert not any(i["code"] == "modemmanager_risk" for i in report["issues"])
    assert "systemctl stop" not in health.summary(report)


def test_occupied_port_lists_pid_without_killing_process(host):
    report = guided(host, owners=[{"pid": 123, "name": "controller"}])
    assert "ps -p 123" in health.summary(report)
    assert any(i["code"] == "port_in_use" for i in report["issues"])
    assert report["ownership_scope"] == "best_effort_visible_processes_only"


def test_unavailable_probe_is_unknown_not_pass(host):
    report = guided(host, probe=lambda _: (None, ""))
    assert report["ports"][0]["modemmanager_ignore"] == "unknown"
    assert any(i["code"] == "udev_check_unavailable" for i in report["issues"])


def test_probe_never_uses_shell_or_unbounded_timeout():
    import subprocess

    import environment_guidance as guide

    with patch.object(
        guide.subprocess, "run", side_effect=subprocess.TimeoutExpired("udevadm", 1)
    ) as run:
        assert guide.query(["udevadm", "info"]) == (None, "")
    assert run.call_args.kwargs["timeout"] == 1
    assert not run.call_args.kwargs.get("shell", False)


def test_environment_guidance_fully_translated_without_altering_commands(host):
    import i18n

    report = guided(
        host,
        writable=False,
        runtime=False,
        probe=lambda _: (None, ""),
        owners=[{"pid": 123, "name": "test"}],
    )
    old = i18n.language()
    try:
        i18n.set_language("en")
        message = i18n.tr(health.summary(report))
        assert not i18n.CJK.search(message)
        assert "\\u" not in message
        for action in report["actions"]:
            assert action["commands"] in message
    finally:
        i18n.set_language(old)
