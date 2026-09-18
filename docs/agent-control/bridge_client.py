"""Official Blender Lab Bridge client. Stdlib only. No MCP."""

from __future__ import annotations

import json
import socket
from typing import Any


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9876
MAX_RESPONSE_BYTES = 8 * 1024 * 1024


class BridgeError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def execute(
    code: str,
    *,
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    connect_timeout: float = 5.0,
    recv_timeout: float = 30.0,
    strict_json: bool = True,
) -> dict[str, Any]:
    """Send one NUL-delimited execute request and return the parsed Bridge object."""
    if "\x00" in code:
        raise BridgeError("invalid_code", "code must not contain a NUL byte")
    request = json.dumps(
        {"type": "execute", "code": code, "strict_json": strict_json},
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8") + b"\0"

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.settimeout(connect_timeout)
        try:
            sock.connect((host, port))
        except ConnectionRefusedError as ex:
            raise BridgeError(
                "connection_refused",
                f"cannot connect {host}:{port}; start SekaiBlender and Local AI Bridge",
            ) from ex
        except TimeoutError as ex:
            raise BridgeError("connect_timeout", f"connect timeout {host}:{port}") from ex
        except OSError as ex:
            raise BridgeError("network_error", f"connect failed: {ex}") from ex

        sock.settimeout(recv_timeout)
        try:
            sock.sendall(request)
        except OSError as ex:
            raise BridgeError("send_failed_ambiguous", f"send interrupted: {ex}") from ex

        buf = bytearray()
        while b"\0" not in buf:
            try:
                chunk = sock.recv(4096)
            except TimeoutError as ex:
                raise BridgeError(
                    "receive_timeout",
                    f"recv timeout after send; treat as unknown, do not retry ({host}:{port})",
                ) from ex
            if not chunk:
                raise BridgeError("incomplete_response", "connection closed before NUL")
            buf.extend(chunk)
            if len(buf) > MAX_RESPONSE_BYTES:
                raise BridgeError(
                    "response_too_large",
                    f"response exceeded {MAX_RESPONSE_BYTES} bytes",
                )

        payload = bytes(buf[: buf.index(b"\0")])
        try:
            response = json.loads(payload)
        except (json.JSONDecodeError, UnicodeDecodeError) as ex:
            raise BridgeError("invalid_response_json", f"invalid JSON: {ex}") from ex
        if not isinstance(response, dict):
            raise BridgeError("invalid_response_shape", "response must be a JSON object")
        return response
    finally:
        sock.close()


def execute_result(code: str, **kwargs: Any) -> dict[str, Any]:
    """Like execute(), but require status=ok and a dict result."""
    response = execute(code, **kwargs)
    if response.get("status") == "error":
        raise BridgeError("blender_error", str(response.get("message", "unknown blender error")))
    if response.get("status") != "ok":
        raise BridgeError("unexpected_status", f"unexpected status: {response.get('status')!r}")
    result = response.get("result")
    if not isinstance(result, dict):
        raise BridgeError("invalid_result", "result must be a dict")
    return result
