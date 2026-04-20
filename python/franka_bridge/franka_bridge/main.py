"""Main entrypoint for franka bridge servers."""

from __future__ import annotations

import argparse
import signal
import threading
from typing import Iterable, List

from .command_server import CommandServer
from .gripper_server import GripperCommandServer, GripperStateServer
from .ros_adapter import RosAdapter
from .state_server import StateServer


def _validate_ports(ports: Iterable[int]) -> None:
    values = list(ports)
    if len(values) != len(set(values)):
        raise ValueError(f"Ports must be unique, got duplicates in: {values}")


def _tcp_bind(host: str, port: int) -> str:
    return f"tcp://{host}:{port}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Franka split-port ZeroMQ bridge")
    parser.add_argument("--bind-host", default="*", help="Bind host for all sockets")
    parser.add_argument("--cmd-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument("--state-hz", type=float, default=20.0)
    parser.add_argument("--enable-gripper-ports", action="store_true")
    parser.add_argument("--gripper-cmd-port", type=int, default=5557)
    parser.add_argument("--gripper-state-port", type=int, default=5558)
    parser.add_argument("--gripper-state-hz", type=float, default=20.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ports: List[int] = [args.cmd_port, args.state_port]
    if args.enable_gripper_ports:
        ports.extend([args.gripper_cmd_port, args.gripper_state_port])
    _validate_ports(ports)

    adapter = RosAdapter()
    command_server = CommandServer(adapter, _tcp_bind(args.bind_host, args.cmd_port))
    state_server = StateServer(
        adapter=adapter,
        bind_address=_tcp_bind(args.bind_host, args.state_port),
        publish_hz=args.state_hz,
    )

    workers: list[threading.Thread] = [
        threading.Thread(target=command_server.run_forever, daemon=True, name="arm-command-server"),
        threading.Thread(target=state_server.run_forever, daemon=True, name="arm-state-server"),
    ]

    gripper_command_server = None
    gripper_state_server = None
    if args.enable_gripper_ports:
        gripper_command_server = GripperCommandServer(
            adapter, _tcp_bind(args.bind_host, args.gripper_cmd_port)
        )
        gripper_state_server = GripperStateServer(
            adapter,
            _tcp_bind(args.bind_host, args.gripper_state_port),
            publish_hz=args.gripper_state_hz,
        )
        workers.extend(
            [
                threading.Thread(
                    target=gripper_command_server.run_forever, daemon=True, name="gripper-command-server"
                ),
                threading.Thread(
                    target=gripper_state_server.run_forever, daemon=True, name="gripper-state-server"
                ),
            ]
        )

    for worker in workers:
        worker.start()

    shutdown = threading.Event()

    def _signal_handler(signum, _frame):  # type: ignore[no-untyped-def]
        del signum
        shutdown.set()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    shutdown.wait()

    state_server.close()
    command_server.close()
    if gripper_state_server:
        gripper_state_server.close()
    if gripper_command_server:
        gripper_command_server.close()


if __name__ == "__main__":
    main()
