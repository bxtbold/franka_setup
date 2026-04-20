"""Client interfaces for split-port franka bridge."""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

import zmq

import msgpack


def _pack(payload: Dict[str, Any]) -> bytes:
    return msgpack.packb(payload, use_bin_type=True)


def _unpack(blob: bytes) -> Dict[str, Any]:
    value = msgpack.unpackb(blob, raw=False)
    if not isinstance(value, dict):
        raise RuntimeError("Unexpected non-dict response from bridge.")
    return value


class _ReqSocket:
    def __init__(self, address: str, timeout_ms: int = 2000) -> None:
        self._address = address
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._sock.connect(address)

    def call(self, command: str, params: Optional[Dict[str, Any]] = None) -> Any:
        request = {"command": command, "params": params or {}}
        self._sock.send(_pack(request))
        try:
            response = _unpack(self._sock.recv())
        except zmq.Again as exc:
            raise RuntimeError(
                f"Request '{command}' timed out waiting for response from {self._address}. "
                "Ensure the matching server port is enabled."
            ) from exc
        if not response.get("ok", False):
            raise RuntimeError(response.get("error", "unknown bridge error"))
        return response.get("result")

    def close(self) -> None:
        self._sock.close(0)
        self._ctx.term()


class _SubSocket:
    def __init__(self, address: str) -> None:
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self._sock.connect(address)

    def recv_latest(self, max_wait_s: float = 0.0) -> Optional[Dict[str, Any]]:
        deadline = time.time() + max_wait_s
        latest = None

        while True:
            try:
                latest = _unpack(self._sock.recv(flags=zmq.NOBLOCK))
            except zmq.Again:
                if latest is not None:
                    return latest
                if max_wait_s <= 0.0 or time.time() >= deadline:
                    return None
                time.sleep(0.005)

    def close(self) -> None:
        self._sock.close(0)
        self._ctx.term()


class FrankaClient:
    def __init__(
        self,
        cmd_address: str = "tcp://127.0.0.1:5555",
        state_address: str = "tcp://127.0.0.1:5556",
        timeout_ms: int = 2000,
    ) -> None:
        self._cmd = _ReqSocket(cmd_address, timeout_ms=timeout_ms)
        self._state = _SubSocket(state_address)

    def ping(self) -> Dict[str, Any]:
        return self._cmd.call("ping")

    def get_state(self) -> Dict[str, Any]:
        return self._cmd.call("get_state")

    def get_latest_state(self, max_wait_s: float = 0.0) -> Optional[Dict[str, Any]]:
        packet = self._state.recv_latest(max_wait_s=max_wait_s)
        if packet is None:
            return None
        return packet.get("state")

    def move_pose(self, position: list[float], orientation: list[float], frame_id: str | None = None) -> Dict[str, Any]:
        params: Dict[str, Any] = {
            "position": position,
            "orientation": orientation,
        }
        if frame_id is not None:
            params["frame_id"] = frame_id
        return self._cmd.call("move_pose", params)

    def joint_reset(self, joints: list[float]) -> Dict[str, Any]:
        return self._cmd.call("joint_reset", {"joints": joints})

    def open_gripper(self) -> Dict[str, Any]:
        return self._cmd.call("open_gripper")

    def close_gripper(self) -> Dict[str, Any]:
        return self._cmd.call("close_gripper")

    def move_gripper(self, position: float) -> Dict[str, Any]:
        return self._cmd.call("move_gripper", {"position": float(position)})

    def close(self) -> None:
        self._cmd.close()
        self._state.close()


class FrankaGripperClient:
    def __init__(
        self,
        cmd_address: str = "tcp://127.0.0.1:5557",
        state_address: str = "tcp://127.0.0.1:5558",
        timeout_ms: int = 2000,
    ) -> None:
        self._cmd = _ReqSocket(cmd_address, timeout_ms=timeout_ms)
        self._state = _SubSocket(state_address)

    def ping(self) -> Dict[str, Any]:
        return self._cmd.call("ping")

    def get_gripper_state(self) -> Dict[str, Any]:
        return self._cmd.call("get_gripper_state")

    def get_latest_state(self, max_wait_s: float = 0.0) -> Optional[Dict[str, Any]]:
        packet = self._state.recv_latest(max_wait_s=max_wait_s)
        if packet is None:
            return None
        return packet.get("state")

    def open_gripper(self) -> Dict[str, Any]:
        return self._cmd.call("open_gripper")

    def close_gripper(self) -> Dict[str, Any]:
        return self._cmd.call("close_gripper")

    def move_gripper(self, position: float) -> Dict[str, Any]:
        return self._cmd.call("move_gripper", {"position": float(position)})

    def close(self) -> None:
        self._cmd.close()
        self._state.close()
