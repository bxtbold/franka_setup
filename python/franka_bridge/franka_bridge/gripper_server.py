"""Optional dedicated gripper command/state servers."""

from __future__ import annotations

import threading
import time
import traceback
from typing import Any, Dict

import zmq

from .protocol import err, ok, pack_message, parse_request, unpack_message
from .ros_adapter import RosAdapter


class GripperCommandServer:
    def __init__(self, adapter: RosAdapter, bind_address: str) -> None:
        self._adapter = adapter
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.REP)
        self._sock.bind(bind_address)

    def _handle(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command, params = parse_request(payload)
        if command == "ping":
            return ok({"status": "pong"})
        if command == "get_gripper_state":
            return ok(self._adapter.get_gripper_state())
        if command == "open_gripper":
            return ok(self._adapter.open_gripper())
        if command == "close_gripper":
            return ok(self._adapter.close_gripper())
        if command == "move_gripper":
            return ok(self._adapter.move_gripper(float(params["position"])))
        raise ValueError(f"Unknown gripper command: {command}")

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


class GripperStateServer:
    def __init__(self, adapter: RosAdapter, bind_address: str, publish_hz: float = 20.0) -> None:
        if publish_hz <= 0:
            raise ValueError("publish_hz must be > 0")
        self._adapter = adapter
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.bind(bind_address)
        self._publish_hz = publish_hz
        self._stop = threading.Event()

    def run_forever(self) -> None:
        interval = 1.0 / self._publish_hz
        while not self._stop.is_set():
            payload = {
                "kind": "gripper_state",
                "timestamp": time.time(),
                "state": self._adapter.get_gripper_state(),
            }
            self._sock.send(pack_message(payload))
            self._stop.wait(interval)

    def stop(self) -> None:
        self._stop.set()

    def close(self) -> None:
        self.stop()
        self._sock.close(0)
        self._ctx.term()
