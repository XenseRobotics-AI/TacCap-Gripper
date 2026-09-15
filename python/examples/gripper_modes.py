#!/usr/bin/env python3
"""Position, compliance and grasp in one session. Moves the physical gripper."""
import argparse
import time

from xense.taccap import FaultError, Gripper


def observe(gripper):
    """Observe a short work interval while the native controller keeps running."""
    for _ in range(10):
        state = gripper.state
        if state.fault:
            raise FaultError(state.fault)
        print(state.control_mode, state.mode, state.position, state.torque_nm)
        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target", nargs="?", help="left, right, firmware SN or device path")
    args = parser.parse_args()

    with Gripper(args.target) as gripper:
        print(gripper.info)
        result = gripper.move(0.6, torque_limit_nm=0.35, speed_rad_s=0.4, wait=True)
        if result.status != "reached":
            raise RuntimeError(f"Position motion stopped: {result.status}")

        gripper.impedance(0.6, kp=5.0, kd=0.5, torque_limit_nm=0.35)
        observe(gripper)  # Compliance has no blocking completion promise.
        gripper.hold(wait=True)

        result = gripper.grasp(torque_nm=0.35, speed_rad_s=0.4, wait=True)
        print(result.status)  # contact is inferred; it does not guarantee a secure grasp.
        if result.status == "contact":
            observe(gripper)  # Perform holding work before release or context exit.

        released = gripper.release(wait=True)
        print("release:", released.status)
        print("Exiting: requesting motor disable; a held object may be released.")


if __name__ == "__main__":
    main()
