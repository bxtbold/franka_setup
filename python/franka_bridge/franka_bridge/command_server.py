"""Command socket server for arm commands."""

from __future__ import annotations

import traceback
from typing import Any, Dict

import zmq

from .protocol import err, ok, pack_message, parse_request, unpack_message
from .ros_adapter import RosAdapter


class CommandServer:
    def __init__(self, adapter: RosAdapter, bind_address: str) -> None:
        self._adapter = adapter
        self._bind_address = bind_address
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.REP)
        self._sock.bind(bind_address)

    def _handle(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command, params = parse_request(payload)

        if command == "ping":
            return ok({"status": "pong"})
        if command == "get_state":
            return ok(self._adapter.get_state())
        if command == "move_pose":
            self._adapter.move_pose(
                position=params["position"],
                orientation=params["orientation"],
                frame_id=params.get("frame_id"),
            )
            return ok({"status": "sent"})
        if command == "joint_reset":
            self._adapter.joint_reset(params["joints"])
            return ok({"status": "sent"})
        if command == "open_gripper":
            return ok(self._adapter.open_gripper())
        if command == "close_gripper":
            return ok(self._adapter.close_gripper())
        if command == "move_gripper":
            return ok(self._adapter.move_gripper(float(params["position"])))
        raise ValueError(f"Unknown command: {command}")

    def run_forever(self) -> None:
        while True:
            try:
                request = unpack_message(self._sock.recv())
                response = self._handle(request)
            except Exception as exc:  # pragma: no cover
                response = err(str(exc), traceback.format_exc())
            self._sock.send(pack_message(response))

    def close(self) -> None:
        self._sock.close(0)
        self._ctx.term()
