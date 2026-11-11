# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Non-blocking TCP socket server that runs inside Blender.

Listens for null-byte-delimited JSON requests, executes Python code
directly in the calling thread, and returns JSON responses.
All socket operations are non-blocking so the server never blocks
Blender's main thread.
"""

__all__ = (
    "DEFAULT_HOST",
    "DEFAULT_PORT",
    "TIMER_INTERVAL_ACTIVE",
    "TIMER_INTERVAL_IDLE",
    "is_running",
    "poll",
    "poll_blocking",
    "start",
    "stop",
    "timer_idle_interval",
    "timer_idle_reset",
    "timer_internal_vars_calc",
    "use_log",
)

import bpy  # pylint: disable=import-error
import json
import math
import queue
import select
import socket
import sys
import threading
import time
import traceback
from collections.abc import Callable
from typing import NamedTuple

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 9876

# Seconds between main-thread timer ticks.
TIMER_INTERVAL_ACTIVE = 0.02
# Seconds between main-thread timer ticks while idle (no pending work).
TIMER_INTERVAL_IDLE = 0.1
# Seconds of inactivity before switching to the idle interval.
_TIMER_INTERVAL_IDLE_DELAY = 5.0


class _TimerState:
    """
    Mutable singleton holding timer-related runtime state.

    This is manipulated from the preferences and updated via ``timer_internal_vars_calc``.
    """

    __slots__ = (
        "interval_active",
        "interval_idle",
        "interval_idle_delay",
        "idle_countdown_reset",
        "idle_countdown",
        "client_timeout_countdown",
    )

    def __init__(self) -> None:
        self.interval_active: float = TIMER_INTERVAL_ACTIVE
        self.interval_idle: float = TIMER_INTERVAL_IDLE
        self.interval_idle_delay: float = _TIMER_INTERVAL_IDLE_DELAY
        # Number of active-rate ticks before switching to idle.
        self.idle_countdown_reset: int = 0
        # Current countdown. When zero, `timer_idle_interval` returns idle.
        self.idle_countdown: int = 0
        # Poll ticks before an idle client is evicted.
        self.client_timeout_countdown: int = 2


_timer = _TimerState()


def timer_internal_vars_calc(
        active: float | None = None,
        idle: float | None = None,
        idle_delay: float | None = None,
) -> None:
    """
    Optionally update ``TIMER_*`` constants and recalculate internal variables.

    When keyword arguments are provided they replace the corresponding
    module-level ``TIMER_*`` value. Pass ``None`` (the default) to leave
    a value unchanged.
    """
    if active is not None:
        _timer.interval_active = active
    if idle is not None:
        _timer.interval_idle = idle
    if idle_delay is not None:
        _timer.interval_idle_delay = idle_delay
    # Round up so the delay is never shorter than requested.
    _timer.idle_countdown_reset = math.ceil(_timer.interval_idle_delay / _timer.interval_active)
    _timer.idle_countdown = _timer.idle_countdown_reset
    _timer.client_timeout_countdown = max(2, math.ceil(_CLIENT_TIMEOUT / _timer.interval_active))


def timer_idle_reset() -> None:
    """
    Signal that work was processed, resetting the idle countdown.
    """
    _timer.idle_countdown = _timer.idle_countdown_reset


def timer_idle_interval() -> float:
    """
    Return the appropriate timer interval, decrementing the idle countdown.

    Returns ``TIMER_INTERVAL_ACTIVE`` while the countdown is positive,
    then ``TIMER_INTERVAL_IDLE`` once it reaches zero.
    """
    if _timer.idle_countdown > 0:
        _timer.idle_countdown -= 1
        return _timer.interval_active
    return _timer.interval_idle


# When True, print every request and response status to STDERR.
use_log: bool = False

# Maximum seconds a single request may occupy Blender's main thread before the
# watchdog interrupts it. ``0`` disables the watchdog. Long imports/bakes may
# need a higher value; an infinite loop otherwise freezes the whole application.
execution_timeout: float = 600.0

_MAX_REQUEST_BYTES = 10 * 1024 * 1024  # 10 MiB.
# Maximum number of queued incoming connections.
_LISTEN_BACKLOG = 128
_RECV_BUFFER_SIZE = 4096
# Seconds before a client that has not sent a complete request is closed.
_CLIENT_TIMEOUT = 10.0
# How often `poll_blocking` checks for shutdown.
_POLL_BLOCKING_TIMEOUT = 1.0
# Maximum time the main thread spends draining queued requests per timer tick.
_SERVICE_BUDGET = 0.02
# Maximum number of requests waiting for main-thread execution.
_REQUEST_QUEUE_MAX = 1024
# Maximum simultaneous sockets owned by the I/O thread (Windows select limit).
_MAX_CLIENTS = 256
_IO_SELECT_TIMEOUT = 0.05
_ACCEPT_BATCH = 64
_DEFERRED_UNSUPPORTED_MESSAGE = (
    "Deferred responses via `check_is_finished` are only supported "
    "by the interactive addon server, and are not available in "
    "background mode. Finish the request synchronously instead."
)

timer_internal_vars_calc()


# ---------------------------------------------------------------------------
# Client connection state.

class _Client:
    """
    Per-connection state for a client (the MCP server process) that has not yet sent a complete request.
    """

    __slots__ = (
        "conn",
        "buffer",
        "timeout",
    )

    def __init__(self, conn: socket.socket) -> None:
        self.conn: socket.socket = conn
        # Accumulates data until the null-byte delimiter is received.
        self.buffer: bytearray = bytearray()
        # Poll ticks remaining before this client is evicted.
        self.timeout: int = _timer.client_timeout_countdown


# ---------------------------------------------------------------------------
# Server state.

class _State:
    """
    Mutable singleton holding the runtime state of this socket server (the Blender add-on side).
    """

    __slots__ = (
        "sock",
        "clients",
    )

    def __init__(self) -> None:
        # The listening socket, or `None` when not running.
        self.sock: socket.socket | None = None
        # Connected clients that have not yet sent a complete request.
        self.clients: list[_Client] = []


_state = _State()


class _ExecResult(NamedTuple):
    """
    Result of executing tool-code.

    When *check_fn* is not ``None``, the caller must defer the response
    and poll the callable for completion (see ``deferred_tool``).
    Otherwise *response* is the final result to send.
    """

    response: dict[str, object]
    check_fn: Callable[[], dict[str, object] | None] | None = None


def _encode_response(response: dict[str, object]) -> bytes:
    """
    Serialize a response dict as null-byte-delimited JSON bytes.
    """
    return (json.dumps(response) + "\0").encode("utf-8")


class _ExecutionTimedOut(BaseException):
    """
    Raised by the watchdog tracer to abort a request that exceeded its deadline.
    """


class _ExecutionWatchdog:
    """
    Interrupt Python code that runs longer than *timeout* seconds.

    Implemented with ``sys.settrace`` so the exception is raised inside the
    executing code and cannot leak outside the guarded ``with`` block.
    C-level calls (rendering, file I/O, etc.) are not interruptible; they are
    bounded by their own duration.
    """

    __slots__ = ("_previous", "_deadline", "_timeout")

    def __init__(self, timeout: float | None) -> None:
        self._previous = None
        self._deadline = time.monotonic() + timeout if timeout and timeout > 0.0 else None
        self._timeout = timeout

    def __enter__(self) -> "_ExecutionWatchdog":
        if self._deadline is None:
            return self
        self._previous = sys.gettrace()
        deadline = self._deadline
        timeout = self._timeout
        counter = 0

        def _tracer(_frame: object, event: str, _arg: object) -> object:
            nonlocal counter
            if event == "line":
                counter += 1
                if counter >= 64:
                    counter = 0
                    if time.monotonic() >= deadline:
                        raise _ExecutionTimedOut(
                            "Execution exceeded the {:g}s bridge limit; "
                            "the request was interrupted to keep SekaiBlender responsive. "
                            "Raise the bridge execution timeout for legitimate long operations.".format(
                                timeout,
                            )
                        )
            return _tracer

        sys.settrace(_tracer)
        return self

    def __exit__(
            self,
            exc_type: type[BaseException] | None,
            exc_val: BaseException | None,
            exc_tb: object,
    ) -> None:
        if self._deadline is not None:
            sys.settrace(self._previous)


def _execute_code(
        code: str,
        strict_json: bool,
        timeout: float | None = None,
) -> _ExecResult:
    """
    Execute *code* and return an ``_ExecResult``.

    :param strict_json: When true, the response *must* be serializable.
        Should always be true, when executing Python code we have full-control over,
        because any non-serializable data is effectively a bug.

        Only allow it to be false when executing arbitrary LLM generated code,
        in this case it's not worth the overhead of correcting the LLM mistake,
        just ``__repr__`` the value so it can fumble its way forward.
    :param timeout: Override for the module-wide ``execution_timeout``.
    """
    from .capture_output import CaptureOutput
    from .weak_sandbox import WeakSandboxForLLM

    effective_timeout = execution_timeout if timeout is None else timeout
    namespace: dict[str, object] = {"result": {}}
    with CaptureOutput() as captured, WeakSandboxForLLM(), _ExecutionWatchdog(effective_timeout):
        try:
            exec(code, namespace)
        except _ExecutionTimedOut as ex:
            response: dict[str, object] = {"status": "error", "message": str(ex)}
            if captured.stdout:
                response["stdout"] = captured.stdout
            if captured.stderr:
                response["stderr"] = captured.stderr
            return _ExecResult(response)
        except Exception:  # pylint: disable=broad-exception-caught
            response = {"status": "error", "message": traceback.format_exc()}
            if captured.stdout:
                response["stdout"] = captured.stdout
            if captured.stderr:
                response["stderr"] = captured.stderr
            return _ExecResult(response)

    # Check for a deferred response (background job in progress).
    check_fn_raw = namespace.get("check_is_finished")
    if check_fn_raw is not None and callable(check_fn_raw):
        check_fn: Callable[[], dict[str, object] | None] = check_fn_raw
        response = {}
        if captured.stdout:
            response["stdout"] = captured.stdout
        if captured.stderr:
            response["stderr"] = captured.stderr
        return _ExecResult(response, check_fn)

    result = namespace["result"]
    if not isinstance(result, dict):
        response = {
            "status": "error",
            "message": (
                "The `result` variable must be a dict, not {:s}. "
                "Wrap your return value: `result = {{\"key\": value}}`"
            ).format(type(result).__name__),
        }
    else:
        # Guard against LLM-generated code storing non-serializable values
        # such as Blender objects, e.g. `result = {"obj": bpy.context.active_object}`.
        # Without this, `json.dumps` fails inside `_encode_response`.
        if strict_json:
            try:
                json.dumps(result)
            except (TypeError, ValueError) as ex:
                response = {
                    "status": "error",
                    "message": "The `result` value is not JSON-serializable: {:s}".format(str(ex)),
                }
            else:
                response = {"status": "ok", "result": result}
        else:
            # Use `repr` as a fallback so non-serializable objects
            # (e.g. Blender ID types) appear as their string representation.
            result = json.loads(json.dumps(result, default=repr))
            response = {"status": "ok", "result": result}
    if captured.stdout:
        response["stdout"] = captured.stdout
    if captured.stderr:
        response["stderr"] = captured.stderr
    return _ExecResult(response)


def _parse_request(
        data: bytes,
) -> tuple[str, bool, float | None] | dict[str, object]:
    """
    Parse a raw request.

    Return ``(code, strict_json, timeout)`` on success, or a response dict on
    error. Safe to call from the I/O thread (no Blender API is touched).
    """

    # Invalid JSON is not expected since the MCP server serializes requests with `json.dumps`.
    # Any error should be rare, the "default" exception path is fine.
    try:
        request = json.loads(data)
    except Exception:  # pylint: disable=broad-exception-caught
        return {"status": "error", "message": traceback.format_exc()}

    if request.get("type") != "execute":
        return {
            "status": "error",
            "message": "Unknown request type: {!r}".format(request.get("type")),
        }
    code = request.get("code", "")
    if not isinstance(code, str):
        return {"status": "error", "message": "The 'code' value must be a string"}

    # Not expected in normal use, but a clear message beats a cryptic trace-back,
    # Also make it clear where the error should be addressed.
    strict_json = request.get("strict_json")
    if not isinstance(strict_json, bool):
        return {
            "status": "error",
            "message": (
                "Internal error: a blender_mcp tool sent a request without the required 'strict_json' boolean key. "
                "This is a bug in the tool that generated this code"
            ),
        }

    request_timeout = request.get("execution_timeout_s")
    if (
        isinstance(request_timeout, bool)
        or not isinstance(request_timeout, (int, float))
        or request_timeout <= 0.0
    ):
        request_timeout = None
    return code, strict_json, request_timeout


def _execute_request(
        code: str,
        strict_json: bool,
        timeout: float | None = None,
) -> _ExecResult:
    """Execute a parsed request and log it when ``use_log`` is enabled."""
    if use_log:
        print("request:\n{:s}".format(code), file=sys.stderr)
    exec_result = _execute_code(code, strict_json=strict_json, timeout=timeout)
    if use_log:
        if exec_result.check_fn is not None:
            print("response: deferred", file=sys.stderr)
        else:
            print("response: {:s}".format(json.dumps(exec_result.response, indent=2)), file=sys.stderr)
    return exec_result


def _execute_code_from_request(
        data: bytes,
) -> tuple[_ExecResult, bool]:
    """
    Parse a raw request and execute it.

    Return ``(exec_result, strict_json)``.
    """
    parsed = _parse_request(data)
    if isinstance(parsed, dict):
        return _ExecResult(parsed), False
    code, strict_json, timeout = parsed
    return _execute_request(code, strict_json, timeout), strict_json


def _close_client(client: _Client) -> None:
    """
    Close a client connection and remove it from the active list.
    """
    try:
        client.conn.close()
    except Exception:  # pylint: disable=broad-exception-caught
        pass
    try:
        _state.clients.remove(client)
    except ValueError:
        pass


# ---------------------------------------------------------------------------
# Polling (called from the execution modules).

def _accept_clients() -> None:
    """
    Accept all pending connections on the listening socket.
    """
    if _state.sock is None:
        return
    while True:
        try:
            conn, _addr = _state.sock.accept()
            conn.setblocking(False)
            _state.clients.append(_Client(conn))
        except BlockingIOError:
            break
        except OSError:
            break


def _service_clients() -> bool:
    """
    Read from all connected clients, execute complete requests.

    Return ``True`` if at least one request was executed.
    """
    did_work = False
    # Iterate over a copy since clients may be removed during the loop.
    for client in _state.clients[:]:
        # Evict clients that have not sent a complete request in time.
        client.timeout -= 1
        if client.timeout <= 0:
            try:
                err: dict[str, object] = {
                    "status": "error",
                    "message": "Client timed out",
                }
                client.conn.sendall(_encode_response(err))
            except OSError:
                pass
            _close_client(client)
            continue

        try:
            chunk = client.conn.recv(_RECV_BUFFER_SIZE)
        except BlockingIOError:
            # No data available yet.
            continue
        except OSError:
            _close_client(client)
            continue

        if not chunk:
            # Client disconnected.
            _close_client(client)
            continue

        client.buffer.extend(chunk)

        # Guard against unbounded input from a misbehaving client.
        if len(client.buffer) > _MAX_REQUEST_BYTES:
            try:
                err = {
                    "status": "error",
                    "message": "Request exceeds {:d} byte limit".format(_MAX_REQUEST_BYTES),
                }
                client.conn.sendall(_encode_response(err))
            except OSError:
                pass
            _close_client(client)
            continue

        if b"\0" not in client.buffer:
            # Request not yet complete.
            continue

        # Execute the request and send the response.
        request_data = bytes(client.buffer[:client.buffer.index(b"\0")])
        try:
            exec_result, strict_json = _execute_code_from_request(request_data)
        except Exception:  # pylint: disable=broad-exception-caught
            exec_result = _ExecResult({"status": "error", "message": traceback.format_exc()})
            strict_json = False

        if exec_result.check_fn is not None:
            # Deferred response: hand the connection to deferred_tool.
            from . import deferred_tool
            deferred_tool.add(
                client.conn,
                exec_result.check_fn,
                strict_json,
                str(exec_result.response.get("stdout", "")),
                str(exec_result.response.get("stderr", "")),
            )
            # Remove from clients without closing the socket.
            try:
                _state.clients.remove(client)
            except ValueError:
                pass
        else:
            try:
                client.conn.sendall(_encode_response(exec_result.response))
            except OSError:
                pass
            _close_client(client)
        did_work = True

    return did_work


# ---------------------------------------------------------------------------
# Interactive mode: socket I/O runs on a background thread, Blender's main
# thread only executes queued code under a per-tick time budget.

class _IOClient:
    """A connection owned by the I/O thread while its request is incomplete."""

    __slots__ = (
        "conn",
        "buffer",
        "deadline",
    )

    def __init__(self, conn: socket.socket) -> None:
        self.conn: socket.socket = conn
        self.buffer: bytearray = bytearray()
        self.deadline: float = time.monotonic() + _CLIENT_TIMEOUT


class _IOState:
    """State shared between the main thread and the socket I/O thread."""

    __slots__ = (
        "sock",
        "thread",
        "stop",
        "clients",
        "requests",
        "writes",
        "pending_writes",
    )

    def __init__(self, sock: socket.socket) -> None:
        self.sock: socket.socket = sock
        self.thread: threading.Thread | None = None
        self.stop = threading.Event()
        # Connections whose request is still being received.
        self.clients: dict[socket.socket, _IOClient] = {}
        # Complete requests waiting for main-thread execution.
        self.requests: queue.Queue = queue.Queue(maxsize=_REQUEST_QUEUE_MAX)
        # Responses waiting to be written, produced by the main thread.
        self.writes: queue.Queue = queue.Queue()
        # Responses currently being written (partial sends included).
        self.pending_writes: dict[socket.socket, list[object]] = {}


_io_state: _IOState | None = None


def _io_close_conn(conn: socket.socket) -> None:
    try:
        conn.close()
    except Exception:  # pylint: disable=broad-exception-caught
        pass


def _queue_response(conn: socket.socket, data: bytes, close: bool) -> None:
    """
    Send a response without blocking Blender's main thread.

    Interactive mode hands the socket back to the I/O thread. Background mode
    and shutdown fall back to a bounded blocking send.
    """
    state = _io_state
    if state is not None and state.thread is not None:
        try:
            state.writes.put_nowait((conn, bytes(data), close))
        except Exception:  # pylint: disable=broad-exception-caught
            _io_close_conn(conn)
        return
    try:
        conn.settimeout(5.0)
        conn.sendall(data)
    except OSError:
        pass
    if close:
        _io_close_conn(conn)


def _io_drain_write_queue(state: _IOState) -> None:
    while True:
        try:
            conn, data, close = state.writes.get_nowait()
        except queue.Empty:
            break
        entry = state.pending_writes.get(conn)
        if entry is None:
            state.pending_writes[conn] = [bytearray(data), close]
        else:
            entry[0].extend(data)  # type: ignore[union-attr]
            entry[1] = bool(entry[1]) or close


def _io_flush_pending(state: _IOState, conn: socket.socket) -> None:
    entry = state.pending_writes.get(conn)
    if entry is None:
        return
    buf: bytearray = entry[0]  # type: ignore[assignment]
    try:
        sent = conn.send(buf)
    except (BlockingIOError, InterruptedError):
        return
    except OSError:
        state.pending_writes.pop(conn, None)
        _io_close_conn(conn)
        return
    if sent:
        del buf[:sent]
    if not buf:
        state.pending_writes.pop(conn, None)
        if entry[1]:
            _io_close_conn(conn)


def _io_accept(state: _IOState) -> None:
    accepted = 0
    while accepted < _ACCEPT_BATCH:
        try:
            conn, _addr = state.sock.accept()
        except BlockingIOError:
            break
        except OSError:
            break
        accepted += 1
        conn.setblocking(False)
        if len(state.clients) >= _MAX_CLIENTS:
            _io_queue_error(state, conn, "Bridge is busy: too many pending clients")
            continue
        state.clients[conn] = _IOClient(conn)


def _io_queue_error(state: _IOState, conn: socket.socket, message: str) -> None:
    state.pending_writes[conn] = [bytearray(_encode_response({
        "status": "error",
        "message": message,
    })), True]


def _io_expire_clients(state: _IOState) -> None:
    now = time.monotonic()
    for conn, client in list(state.clients.items()):
        if now > client.deadline:
            state.clients.pop(conn, None)
            _io_queue_error(state, conn, "Client timed out")


def _io_read_client(state: _IOState, client: _IOClient) -> None:
    conn = client.conn
    try:
        chunk = conn.recv(_RECV_BUFFER_SIZE)
    except BlockingIOError:
        return
    except OSError:
        state.clients.pop(conn, None)
        _io_close_conn(conn)
        return

    if not chunk:
        state.clients.pop(conn, None)
        _io_close_conn(conn)
        return

    client.buffer.extend(chunk)
    if len(client.buffer) > _MAX_REQUEST_BYTES:
        state.clients.pop(conn, None)
        _io_queue_error(state, conn, "Request exceeds {:d} byte limit".format(_MAX_REQUEST_BYTES))
        return
    if b"\0" not in client.buffer:
        return

    request_data = bytes(client.buffer[:client.buffer.index(b"\0")])
    # Hand ownership to the main thread: stop reading this connection.
    state.clients.pop(conn, None)

    parsed = _parse_request(request_data)
    if isinstance(parsed, dict):
        state.pending_writes[conn] = [bytearray(_encode_response(parsed)), True]
        return
    code, strict_json, timeout = parsed
    try:
        state.requests.put_nowait((conn, code, strict_json, timeout))
    except queue.Full:
        _io_queue_error(state, conn, "Bridge is busy; retry shortly")


def _io_thread_main(state: _IOState) -> None:
    try:
        while not state.stop.is_set():
            _io_drain_write_queue(state)
            read_list: list[socket.socket] = [state.sock]
            read_list.extend(state.clients.keys())
            # Pending write sockets are not read: the client is done sending.
            write_list = list(state.pending_writes.keys())
            try:
                readable, writable, _errored = select.select(
                    read_list, write_list, [], _IO_SELECT_TIMEOUT,
                )
            except (OSError, ValueError):
                if state.stop.is_set():
                    break
                client_count = len(state.clients)
                if client_count == 0 and not state.pending_writes:
                    time.sleep(0.05)
                continue

            for conn in writable:
                _io_flush_pending(state, conn)

            if state.sock in readable:
                _io_accept(state)

            for conn in readable:
                if conn is state.sock:
                    continue
                client = state.clients.get(conn)
                if client is not None:
                    _io_read_client(state, client)

            _io_expire_clients(state)
    finally:
        try:
            state.sock.close()
        except Exception:  # pylint: disable=broad-exception-caught
            pass
        for conn in list(state.clients):
            _io_close_conn(conn)
        state.clients.clear()
        for conn in list(state.pending_writes):
            _io_close_conn(conn)
        state.pending_writes.clear()
        while True:
            try:
                conn, _code, _strict_json, _timeout = state.requests.get_nowait()
            except queue.Empty:
                break
            _io_close_conn(conn)


def _conn_is_closed(conn: socket.socket) -> bool:
    """Best-effort check for a client that disconnected while queued."""
    try:
        return conn.recv(1, socket.MSG_PEEK) == b""
    except BlockingIOError:
        return False
    except OSError:
        return True


def poll() -> bool:
    """
    Non-blocking poll: drain queued requests, execute them with a bounded
    main-thread budget and check deferred responses.

    Return ``True`` if work was done or deferred/queued work remains.
    """
    from . import deferred_tool

    state = _io_state
    if state is not None and state.thread is not None:
        did_work = False
        deadline = time.monotonic() + _SERVICE_BUDGET
        while True:
            try:
                conn, code, strict_json, timeout = state.requests.get_nowait()
            except queue.Empty:
                break
            if _conn_is_closed(conn):
                _io_close_conn(conn)
                did_work = True
                continue
            exec_result = _execute_request(code, strict_json, timeout)
            if exec_result.check_fn is not None:
                deferred_tool.add(
                    conn,
                    exec_result.check_fn,
                    strict_json,
                    str(exec_result.response.get("stdout", "")),
                    str(exec_result.response.get("stderr", "")),
                )
            else:
                _queue_response(conn, _encode_response(exec_result.response), True)
            did_work = True
            if time.monotonic() >= deadline:
                break

        if deferred_tool.poll():
            did_work = True
        if deferred_tool.has_pending() or not state.requests.empty():
            did_work = True
        return did_work

    _accept_clients()
    did_work = _service_clients()
    if deferred_tool.poll():
        did_work = True
    # Stay in active polling mode while deferred clients exist.
    if deferred_tool.has_pending():
        did_work = True
    return did_work


def _handle_blocking_client(conn: socket.socket) -> bool:
    """
    Handle a single client connection synchronously with blocking I/O.

    Return ``True`` if a request was executed.
    """
    conn.settimeout(_CLIENT_TIMEOUT)
    try:
        buf = bytearray()
        while b"\0" not in buf:
            chunk = conn.recv(_RECV_BUFFER_SIZE)
            if not chunk:
                # Client disconnected.
                return False
            buf.extend(chunk)
            if len(buf) > _MAX_REQUEST_BYTES:
                err: dict[str, object] = {
                    "status": "error",
                    "message": "Request exceeds {:d} byte limit".format(_MAX_REQUEST_BYTES),
                }
                conn.sendall(_encode_response(err))
                return False

        request_data = bytes(buf[:buf.index(b"\0")])
        try:
            exec_result, _strict_json = _execute_code_from_request(request_data)
            if exec_result.check_fn is not None:
                # Unpack to preserve stdout/stderr captured before the deferred handler was set up.
                response = {**exec_result.response, "status": "error", "message": _DEFERRED_UNSUPPORTED_MESSAGE}
                exec_result = _ExecResult(response)
        except Exception:  # pylint: disable=broad-exception-caught
            exec_result = _ExecResult({"status": "error", "message": traceback.format_exc()})
        conn.sendall(_encode_response(exec_result.response))
        return True
    except socket.timeout:
        try:
            err = {"status": "error", "message": "Client timed out"}
            conn.sendall(_encode_response(err))
        except OSError:
            pass
        return False
    except OSError:
        return False
    finally:
        conn.close()


def poll_blocking(timeout: float = _POLL_BLOCKING_TIMEOUT) -> bool:
    """
    Block until a connection arrives (up to *timeout* seconds), then
    handle it synchronously with blocking I/O.

    For use in background mode where the GUI is not running.
    Return ``True`` if a request was executed.
    """
    if _state.sock is None:
        return False

    try:
        readable, _writable, _errored = select.select([_state.sock], [], [], timeout)
    except (OSError, ValueError):
        return False

    if not readable:
        return False

    try:
        conn, _addr = _state.sock.accept()
    except (BlockingIOError, OSError):
        return False

    return _handle_blocking_client(conn)


# ---------------------------------------------------------------------------
# Public API.

def start(host: str, port: int) -> None:
    """
    Bind the listening socket and begin accepting connections.

    This does not block. The caller must arrange for ``poll`` to be
    called periodically (see ``execute_interactive`` and
    ``execute_blocking``).

    Interactive sessions run socket I/O on a background thread so connection
    floods cannot stall Blender's main thread.

    Callers should catch ``Exception`` broadly rather than specific types,
    since failures may be:
    - ``RuntimeError``, e.g. server already running.
    - ``OSError``, e.g. address already in use.
    ...other exceptions that are difficult to predict exhaustively.
    """
    global _io_state

    if is_running():
        raise RuntimeError("Server is already running")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setblocking(False)
        sock.bind((host, port))
        sock.listen(_LISTEN_BACKLOG)
    except OSError:
        sock.close()
        raise

    _state.sock = sock

    # Background mode has no timers: keep the synchronous select loop.
    if bpy.app.background:
        return

    state = _IOState(sock)
    thread = threading.Thread(
        target=_io_thread_main,
        args=(state,),
        name="MCP-Bridge-IO",
        daemon=True,
    )
    state.thread = thread
    _io_state = state
    try:
        thread.start()
    except Exception:  # pylint: disable=broad-exception-caught
        _io_state = None
        _state.sock = None
        sock.close()
        raise


def stop() -> None:
    """
    Close the listening socket, all client connections, and deferred responses.
    """
    global _io_state

    from . import deferred_tool

    state = _io_state
    if state is not None:
        _io_state = None
        state.stop.set()
        try:
            state.sock.close()
        except Exception:  # pylint: disable=broad-exception-caught
            pass
        thread = state.thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=2.0)
        # The thread normally closes these; close any survivors defensively.
        for conn in list(state.clients):
            _io_close_conn(conn)
        state.clients.clear()
        for conn in list(state.pending_writes):
            _io_close_conn(conn)
        state.pending_writes.clear()
        while True:
            try:
                conn, _code, _strict_json, _timeout = state.requests.get_nowait()
            except queue.Empty:
                break
            _io_close_conn(conn)
        _state.sock = None
        deferred_tool.close_all()
        return

    sock = _state.sock
    _state.sock = None
    if sock is not None:
        try:
            sock.close()
        except Exception:  # pylint: disable=broad-exception-caught
            pass

    for client in _state.clients:
        try:
            client.conn.close()
        except Exception:  # pylint: disable=broad-exception-caught
            pass
    _state.clients.clear()

    deferred_tool.close_all()


def is_running() -> bool:
    """
    Return whether the server is currently listening.
    """
    return _state.sock is not None
