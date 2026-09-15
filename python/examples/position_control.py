#!/usr/bin/env python3
"""Interactive position following and timestamped feedback capture.

Examples (these move hardware; quit/Ctrl+C disables):
  python python/examples/position_control.py left --close-compensation-rad 0.03
  python python/examples/position_control.py left --sequence 1,0,1 --dwell 5 --csv /tmp/gripper.csv
"""
import argparse
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import threading
import time

from xense.taccap import Gripper


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("device", nargs="?", default="left")
    parser.add_argument("--torque", type=float, default=.35, help="Motor torque budget in Nm")
    parser.add_argument("--speed", type=float, default=.5, help="Reference slew in motor rad/s")
    parser.add_argument("--hz", type=int, default=100, choices=(20, 25, 40, 50, 100))
    parser.add_argument("--close-compensation-rad", type=float, default=0.)
    parser.add_argument("--csv", type=Path, help="Record fresh feedback, host timestamps and target errors")
    parser.add_argument("--sequence", help="Optional timed opening targets, e.g. 1,0,1")
    parser.add_argument("--dwell", type=float, default=5., help="Seconds per sequence target, not an arrival timeout")
    args = parser.parse_args()
    targets = None
    if args.sequence:
        try:
            targets = [float(v) for v in args.sequence.split(",")]
        except ValueError:
            parser.error("sequence must contain comma-separated positions")
        if not all(math.isfinite(v) and 0 <= v <= 1 for v in targets):
            parser.error("sequence positions must be in [0,1]")
    if not math.isfinite(args.dwell) or args.dwell <= 0:
        parser.error("dwell must be positive and finite")
    stopped = threading.Event()
    errors = []

    with Gripper(args.device, torque_limit_nm=args.torque, speed_rad_s=args.speed,
                 update_hz=args.hz, close_compensation_rad=args.close_compensation_rad) as gripper:
        print(gripper.info)
        input("Clear the motion area. Press Enter to enable position control. ")
        gripper.start()
        command_lock = threading.Lock()
        command_issued_s = None

        def set_position(target):
            nonlocal command_issued_s
            with command_lock:
                issued = time.monotonic()
                gripper.set_position(target)
                command_issued_s = issued

        def record():
            try:
                with args.csv.open("w", newline="") as output:
                    writer = csv.writer(output)
                    writer.writerow(["host_monotonic_s", "command_issued_monotonic_s", "seq", "command_id", "ready", "target", "position",
                                     "target_error_rad", "velocity_rad_s", "torque_nm", "temperature_c",
                                     "age_ms", "effective_torque_limit_nm", "target_limited", "fault"])
                    last_seq = -1
                    while not stopped.is_set():
                        with command_lock:
                            s = gripper.state
                            issued = command_issued_s
                        if s.seq != last_seq:
                            writer.writerow([time.monotonic(), issued, s.seq, s.command_id, s.ready, s.target, s.position,
                                             s.target_error_rad, s.velocity_rad_s, s.torque_nm, s.temperature_c,
                                             s.age_ms, s.effective_torque_limit_nm, s.target_limited, s.fault])
                            last_seq = s.seq
                        if s.fault:
                            raise RuntimeError(s.fault)
                        stopped.wait(1 / args.hz / 2)
            except Exception as exc:
                errors.append(exc)
                stopped.set()

        recorder = threading.Thread(target=record, name="gripper-csv") if args.csv else None
        if recorder:
            recorder.start()
        try:
            if targets is not None:
                for target in targets:
                    if errors:
                        raise errors[0]
                    print(f"Target {target:.3f}")
                    set_position(target)
                    deadline = time.monotonic() + args.dwell
                    while time.monotonic() < deadline:
                        state = gripper.state
                        if state.fault:
                            raise RuntimeError(state.fault)
                        if stopped.wait(.02):
                            raise errors[0]
                    print(gripper.state)
            else:
                print("Enter opening 0..1; s = status; q = quit. Control keeps the last target.")
                while not stopped.is_set():
                    value = input("position> ").strip().lower()
                    if value == "q":
                        break
                    if value == "s":
                        print(gripper.state)
                        print(json.dumps(asdict(gripper.timing), indent=2))
                        continue
                    try:
                        set_position(float(value))
                    except ValueError as exc:
                        print(exc)
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            stopped.set()
            if recorder:
                recorder.join()
            print(json.dumps(asdict(gripper.timing), indent=2))
            print("Exiting: disabling the motor; support any held object.")
        if errors:
            raise errors[0]


if __name__ == "__main__":
    main()
