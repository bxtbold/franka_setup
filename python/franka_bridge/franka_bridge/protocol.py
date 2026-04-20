"""Wire protocol helpers for franka bridge."""

from __future__ import annotations

from typing import Any, Dict

import msgpack


def pack_message(payload: Dict[str, Any]) -> bytes:
    return msgpack.packb(payload, use_bin_type=True)


def unpack_message(blob: bytes) -> Dict[str, Any]:
    value = msgpack.unpackb(blob, raw=False)
    if not isinstance(value, dict):
        raise ValueError("Expected message payload to be a dictionary.")
    return value


def ok(result: Any = None) -> Dict[str, Any]:
    return {"ok": True, "result": result}


def err(message: str, details: str | None = None) -> Dict[str, Any]:
    payload = {"ok": False, "error": message}
    if details:
        payload["details"] = details
    return payload


def parse_request(payload: Dict[str, Any]) -> tuple[str, Dict[str, Any]]:
    command = payload.get("command")
    if not isinstance(command, str) or not command:
        raise ValueError("Request must include a non-empty string 'command'.")

    params = payload.get("params", {})
    if params is None:
        params = {}
    if not isinstance(params, dict):
        raise ValueError("Request 'params' must be a dictionary.")
    return command, params
