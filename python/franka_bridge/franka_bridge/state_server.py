"""State stream publisher server."""

from __future__ import annotations

import threading
import time

import zmq

from .protocol import pack_message
from .ros_adapter import RosAdapter


class StateServer:
    def __init__(self, adapter: RosAdapter, bind_address: str, publish_hz: float = 20.0) -> None:
        if publish_hz <= 0:
            raise ValueError("publish_hz must be > 0")
        self._adapter = adapter
        self._bind_address = bind_address
        self._publish_hz = publish_hz
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.bind(bind_address)
        self._stop = threading.Event()

    def run_forever(self) -> None:
        interval = 1.0 / self._publish_hz
        while not self._stop.is_set():
            payload = {
                "kind": "arm_state",
                "timestamp": time.time(),
                "state": self._adapter.get_state(),
            }
            self._sock.send(pack_message(payload))
            self._stop.wait(interval)

    def stop(self) -> None:
        self._stop.set()

    def close(self) -> None:
        self.stop()
        self._sock.close(0)
        self._ctx.term()
