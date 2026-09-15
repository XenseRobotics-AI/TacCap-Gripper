#!/usr/bin/env python3
"""Daily follower operations. Running this example moves the physical gripper."""
import argparse

from xense.taccap import Gripper, TimeoutError


def run_motion(gripper, method, *args, **kwargs):
    try:
        return method(*args, **kwargs)
    except TimeoutError as exc:
        print(f"Motion timeout: {exc}")
        print("Feedback:", getattr(exc, "state", None))
        try:
            print("Firmware execution:", gripper.diagnostics())
        except Exception as diagnostic_error:  # noqa: BLE001 - preserve the original motion timeout.
            print("Could not read diagnostics:", diagnostic_error)
        print("Exiting this example now requests disable; support any held object.")
        raise



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target", nargs="?", help="left, right, firmware SN or device path")
    parser.add_argument("--position", type=float, default=1.0, help="opening in [0,1]")
    parser.add_argument("--grasp", action="store_true", help="close until contact or fully closed")
    parser.add_argument("--hold", action="store_true",
                        help="with --grasp: open for loading, then hold until Enter releases")
    parser.add_argument("--torque", type=float, default=0.35, help="motor torque in Nm")
    parser.add_argument("--release-torque", type=float, default=0.35,
                        help="opening torque limit in Nm, independent of grasp torque")
    parser.add_argument("--release-timeout", type=float, default=None,
                        help="opening timeout in seconds (defaults to --timeout)")
    parser.add_argument("--speed", type=float, default=0.5, help="motor reference speed in rad/s")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--list", action="store_true", help="discover followers without motion")
    args = parser.parse_args()
    if args.list:
        for device in Gripper.discover():
            print(device.firmware_sn, device.side, device.device)
        return
    if args.hold and not args.grasp:
        parser.error("--hold requires --grasp")
    release_timeout = args.timeout if args.release_timeout is None else args.release_timeout
    if release_timeout <= 0:
        parser.error("--release-timeout must be positive")
    with Gripper(args.target) as gripper:
        if args.hold:
            opened = run_motion(gripper, gripper.release, torque_limit_nm=args.release_torque, speed_rad_s=args.speed,
                                     wait=True, timeout=release_timeout)
            if opened.status != "reached":
                raise RuntimeError(f"Cannot open for loading: {opened.status}")
            input("Jaws open. Place the object, remove your fingers, then press Enter to grasp. ")
        if args.grasp:
            result = run_motion(gripper, gripper.grasp,torque_nm=args.torque, speed_rad_s=args.speed,
                                   wait=True, timeout=args.timeout)
        else:
            result = run_motion(gripper, gripper.move,args.position, torque_limit_nm=args.torque,
                                  speed_rad_s=args.speed,
                                  wait=True, timeout=args.timeout)
        print(result.operation, result.status, result.state)
        if args.hold:
            if result.status == "contact":
                input("Contact detected; holding (not a guarantee of a secure grasp). "
                      "Support the object and press Enter to release. Ctrl+C disables. ")
            else:
                print("No object confirmed; reopening instead of continuing to grip.")
            released = run_motion(gripper, gripper.release, torque_limit_nm=args.release_torque, speed_rad_s=args.speed,
                                       wait=True, timeout=release_timeout)
            print("release:", released.status)
        print("Exiting: requesting motor disable; a held object may be released.")


if __name__ == "__main__":
    main()
