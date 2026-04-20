#!/usr/bin/env python3
"""Smoke test for split-port franka bridge."""

from __future__ import annotations

import argparse
import json
import time

from franka_bridge_client import FrankaClient, FrankaGripperClient


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Smoke test franka bridge client")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--cmd-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument("--gripper-cmd-port", type=int, default=5557)
    parser.add_argument("--gripper-state-port", type=int, default=5558)
    parser.add_argument("--test-gripper", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    client = FrankaClient(
        cmd_address=f"tcp://{args.host}:{args.cmd_port}",
        state_address=f"tcp://{args.host}:{args.state_port}",
    )
    try:
        print("ping:", client.ping())
        print("get_state:", json.dumps(client.get_state(), indent=2))

        print("waiting for state stream...")
        state = client.get_latest_state(max_wait_s=1.0)
        print("stream_state:", json.dumps(state, indent=2))
    finally:
        client.close()

    if args.test_gripper:
        grip = FrankaGripperClient(
            cmd_address=f"tcp://{args.host}:{args.gripper_cmd_port}",
            state_address=f"tcp://{args.host}:{args.gripper_state_port}",
        )
        try:
            print("gripper_ping:", grip.ping())
            print("gripper_state:", json.dumps(grip.get_gripper_state(), indent=2))
            time.sleep(0.2)
            print("gripper_stream:", json.dumps(grip.get_latest_state(max_wait_s=1.0), indent=2))
        finally:
            grip.close()


if __name__ == "__main__":
    main()
