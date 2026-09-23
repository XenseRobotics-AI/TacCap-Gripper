# Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
"""Shared plumbing for the example CLIs: which gripper, and how to print it.

Every runnable example in this directory imports this. It is not a library and
not part of ``xense.taccap`` — it is the one place the examples agree on how a
user names a gripper on the command line, so that ``left`` means the same thing
in all of them.

It used to live in ``_calib_flow.py``, which was the wrong home in a way that
cost people time: ``impedance_control.py`` imported something called
"calib flow" purely to get a ``left|right|SN`` argument, which reads like a
mistake. The calibration walkthrough is still in ``_calib_flow.py``; only three
examples need it, and it imports from here.

What is here:
  - the ``left`` / ``right`` / SN selector — ``add_target_argument()``,
    ``resolve_target()``, ``open_follower()``
  - version rendering — ``format_version()``, ``firmware_version()``
  - terminal colour and unit helpers used by the prompts
"""

from __future__ import annotations

import math
import sys

from xense.taccap import Side, scan_grippers

SIDE_ALIASES = {
    "left": Side.Left,
    "l": Side.Left,
    "right": Side.Right,
    "r": Side.Right,
}


def side_str(side) -> str:
    return {Side.Left: "Left", Side.Right: "Right"}.get(side, "Unknown")


def listing(all_eps) -> str:
    return (", ".join(f"{e.firmware_sn} ({side_str(e.side)})" for e in all_eps)
            or "(none)")


def resolve_target(target: str | None):
    """``left`` / ``right`` / a firmware SN / ``None`` -> one endpoint.

    ``None`` means "the only gripper plugged in" and fails loudly when that is
    not what is plugged in — an example must never guess which half of a
    bilateral rig the user meant.

    Returns ``(endpoint, by_side, all_endpoints)``.
    """
    all_eps = scan_grippers()

    if target is None:
        if len(all_eps) == 1:
            return all_eps[0], False, all_eps
        sys.exit(
            f"error: {len(all_eps)} grippers plugged in — say which one: "
            "'left', 'right', or a firmware SN.\n"
            f"       currently visible: {listing(all_eps)}"
        )

    side = SIDE_ALIASES.get(target.strip().lower())
    by_side = side is not None
    if by_side:
        matches = [e for e in all_eps if e.side == side]
        what = f"side={side_str(side)}"
    else:
        matches = [e for e in all_eps if e.firmware_sn == target]
        what = f"firmware SN={target!r}"

    if not matches:
        hint = "" if by_side else "\n       (you can also just pass 'left' or 'right')"
        sys.exit(
            f"error: no gripper matching {what} is plugged in.\n"
            f"       currently visible: {listing(all_eps)}{hint}"
        )
    if len(matches) > 1:
        if by_side:
            sys.exit(
                f"error: {len(matches)} plugged-in grippers report {what} — "
                f"their firmware SNs are "
                f"{', '.join(e.firmware_sn for e in matches)}. Fix the burned "
                "SNs, or pass the SN you want explicitly."
            )
        sys.exit(
            f"error: {len(matches)} grippers report SN={target!r} — "
            "firmware-SN collision, check firmware burning."
        )
    return matches[0], by_side, all_eps


def add_target_argument(parser, *, required: bool = False) -> None:
    """Attach the repo-wide positional selector to an argparse parser."""
    parser.add_argument(
        "target", nargs=None if required else "?", metavar="left|right|SN",
        help="Which gripper: 'left' / 'right' (side comes from the firmware SN), "
             "or an explicit SN such as TCGU01A28Z0116m."
             + ("" if required else " Omit when exactly one is plugged in."),
    )


def open_follower(target):
    """``resolve_target()`` + ``FollowerGripper`` -> ``(gripper, endpoint)``.

    Three examples had a byte-identical copy of this, each with its own
    ``--side`` flag, which contradicted the single positional selector the rest
    of the examples use (see docs/EXAMPLES.md). One home, one convention.

    The endpoint comes back with the gripper because callers want its SN and
    side for display, and re-deriving them would mean a second ``scan_grippers()``
    -- a full USB walk, and a chance for the answer to change in between.
    """
    from xense.taccap import FollowerGripper

    endpoint, _by_side, _all = resolve_target(target)
    print(f"[discovery] {endpoint.firmware_sn}  {endpoint.mcu_device}")
    return FollowerGripper(mcu_device=endpoint.mcu_device), endpoint


_TTY = sys.stdout.isatty()


def _c(code: str, s: str) -> str:
    return f"\033[{code}m{s}\033[0m" if _TTY else s


def bold(s):
    return _c("1", s)


def yellow(s):
    return _c("33", s)


def green(s):
    return _c("32", s)


def red(s):
    return _c("31", s)


def deg(rad: float) -> str:
    return f"{math.degrees(rad):.1f}°"


def format_version(major, minor, patch, build=None) -> str:
    """Render a firmware version for humans: MAJOR.MINOR.PATCH.

    The wire carries a fourth "build" byte, but it is pinned to 0 and means
    nothing to users, so it is deliberately not shown — printing "1.2.1.0"
    only invited people to type the trailing zero into version comparisons.
    `build` is accepted so callers can pass all four through without
    unpacking, and ignored on purpose. Mirrors C++
    protocol::version_string().
    """
    return f"{major}.{minor}.{patch}"


def firmware_version(gripper) -> str:
    """Best-effort MAJOR.MINOR.PATCH, for capability error messages.

    Cmd::GetVersion returns the compiled-in constant, not the OTA bank
    metadata, so this is the authoritative "what build am I talking to".
    """
    from xense.taccap import Cmd

    try:
        ack = gripper.transport.send_cmd(Cmd.GetVersion, b"", 500)
        if len(ack.data) >= 4:
            return format_version(*ack.data[:4])
    except Exception:
        pass
    return "<unknown>"
