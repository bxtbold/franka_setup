#!/usr/bin/env python3
"""Interactive safety-first controller test for Franka bridge."""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import List

from franka_bridge_client import FrankaClient

# Conservative Panda joint limits (rad).
JOINT_MIN = [-2.89, -1.76, -2.89, -3.07, -2.89, -0.01, -2.89]
JOINT_MAX = [2.89, 1.76, 2.89, -0.07, 2.89, 3.75, 2.89]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Safe Franka controller test")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--cmd-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument("--observe-seconds", type=float, default=3.0)
    parser.add_argument(
        "--mode",
        choices=["cartesian", "joint"],
        default="cartesian",
        help="motion test mode; cartesian matches the default impedance controller",
    )
    parser.add_argument("--axis", choices=["x", "y", "z"], default="z", help="axis for cartesian nudge")
    parser.add_argument("--delta-m", type=float, default=0.01, help="tiny cartesian delta in meters")
    parser.add_argument("--joint-index", type=int, default=6, help="0-based joint index")
    parser.add_argument("--delta-rad", type=float, default=0.01, help="tiny joint delta in radians")
    parser.add_argument("--allow-motion", action="store_true", help="enable the tiny motion phase")
    return parser.parse_args()


def _prompt_yes(prompt: str) -> bool:
    value = input(f"{prompt} [y/N]: ").strip().lower()
    return value in {"y", "yes"}


def _validate_joints(joints: List[float]) -> None:
    if len(joints) != 7:
        raise RuntimeError(f"Expected 7 joints, got {len(joints)}")


def _within_limits(q: List[float]) -> bool:
    for i, val in enumerate(q):
        if val < JOINT_MIN[i] or val > JOINT_MAX[i]:
            return False
    return True


def main() -> None:
    args = parse_args()
    if args.joint_index < 0 or args.joint_index > 6:
        raise SystemExit("--joint-index must be in [0, 6]")
    if abs(args.delta_rad) > 0.03:
        raise SystemExit("--delta-rad too large for safe test. Use <= 0.03")
    if abs(args.delta_m) > 0.01:
        raise SystemExit("--delta-m too large for safe test. Use <= 0.01")

    client = FrankaClient(
        cmd_address=f"tcp://{args.host}:{args.cmd_port}",
        state_address=f"tcp://{args.host}:{args.state_port}",
        timeout_ms=3000,
    )

    try:
        print("=== Connectivity Check ===")
        print("ping:", client.ping())
        state = client.get_state()
        print("initial_state:", json.dumps(state, indent=2))

        joints = state.get("joint_positions") or []
        _validate_joints(joints)

        print(f"\n=== Observe Stream ({args.observe_seconds:.1f}s) ===")
        start = time.time()
        samples = 0
        while time.time() - start < args.observe_seconds:
            packet = client.get_latest_state(max_wait_s=0.2)
            if packet is not None:
                samples += 1
            time.sleep(0.01)
        print(f"stream_samples={samples}")
        if samples == 0:
            raise RuntimeError("No state stream samples received; aborting motion test.")

        if not args.allow_motion:
            print("\nMotion phase is disabled. Re-run with --allow-motion when ready.")
            return

        print("\n=== Tiny Motion Phase ===")
        print("Safety reminder: Keep E-stop accessible. Ensure workspace is clear.")
        if not _prompt_yes(f"Proceed with tiny {args.mode} nudge test?"):
            print("Motion canceled by user.")
            return

        if args.mode == "joint":
            q_home = [float(v) for v in joints]
            q_target = q_home.copy()
            q_target[args.joint_index] += args.delta_rad

            if not _within_limits(q_target):
                raise RuntimeError("Target joint command exceeds conservative joint limits; aborting.")

            print(f"Sending tiny joint nudge: joint[{args.joint_index}] += {args.delta_rad:.4f} rad")
            client.joint_reset(q_target)
            time.sleep(0.5)
        else:
            ee_pose = state.get("ee_pose")
            if not isinstance(ee_pose, dict):
                raise RuntimeError(
                    "No ee_pose available in state. If needed, use --mode joint, "
                    "or ensure franka_state topics are available to the bridge."
                )
            pos = [float(v) for v in ee_pose.get("position", [])]
            quat = [float(v) for v in ee_pose.get("orientation", [])]
            if len(pos) != 3 or len(quat) != 4:
                raise RuntimeError("Invalid ee_pose in state. Need position(3) and orientation(4).")

            axis_to_idx = {"x": 0, "y": 1, "z": 2}
            idx = axis_to_idx[args.axis]
            pos_target = pos.copy()
            pos_target[idx] += args.delta_m
            print(f"Sending tiny cartesian nudge: {args.axis} += {args.delta_m:.4f} m")
            client.move_pose(position=pos_target, orientation=quat, frame_id=ee_pose.get("frame_id"))
            time.sleep(0.5)

        if not _prompt_yes("Return to start joint configuration now?"):
            print("Skipped automatic return by user request.")
            return

        if args.mode == "joint":
            print("Returning to start joint configuration...")
            client.joint_reset(q_home)
            time.sleep(0.5)
        else:
            print("Returning to start cartesian pose...")
            client.move_pose(position=pos, orientation=quat, frame_id=ee_pose.get("frame_id"))
            time.sleep(0.5)

        final_state = client.get_state()
        print("final_state:", json.dumps(final_state, indent=2))
        print("\nSafe controller test completed.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(1)
    finally:
        client.close()


if __name__ == "__main__":
    main()
