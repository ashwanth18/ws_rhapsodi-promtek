"""TCP/JSON client for the Schneider Lexium Cobot controller.

Two independent connections are used, exactly as documented in the
LexiumCobotCommunication protocol:

* Command channel  -> TCP port 10001. Strict request/response: send one JSON
  command, wait for the JSON acknowledgement, then send the next.
* Feedback channel -> TCP port 10000. The controller streams a JSON status
  object roughly every 20 ms. We keep only the latest decoded object.

Both channels frame messages as bare, concatenated JSON objects (no newline
delimiter), so we decode incrementally with ``json.JSONDecoder.raw_decode``.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Callable, Dict, Optional


class LexiumError(Exception):
    """Raised when the controller acknowledges a command with an error."""


class LexiumClient:
    """Thread-safe client for the Lexium command and feedback channels."""

    def __init__(
        self,
        ip: str,
        command_port: int = 10001,
        feedback_port: int = 10000,
        connect_timeout: float = 5.0,
        command_timeout: float = 10.0,
        reconnect_backoff: float = 1.0,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._ip = ip
        self._command_port = command_port
        self._feedback_port = feedback_port
        self._connect_timeout = connect_timeout
        self._command_timeout = command_timeout
        self._reconnect_backoff = reconnect_backoff
        self._log = logger or (lambda msg: None)

        # Command channel.
        self._cmd_sock: Optional[socket.socket] = None
        self._cmd_buf = b""
        self._cmd_lock = threading.Lock()
        self._decoder = json.JSONDecoder()

        # Feedback channel.
        self._fb_state: Optional[Dict] = None
        self._fb_stamp: float = 0.0
        self._fb_lock = threading.Lock()

        self._running = False
        self._fb_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        """Open both channels and start the feedback reader thread."""
        self._running = True
        self._connect_command()
        self._fb_thread = threading.Thread(
            target=self._feedback_loop, name="lexium_feedback", daemon=True
        )
        self._fb_thread.start()

    def stop(self) -> None:
        self._running = False
        with self._cmd_lock:
            self._close(self._cmd_sock)
            self._cmd_sock = None
        if self._fb_thread is not None:
            self._fb_thread.join(timeout=2.0)

    @staticmethod
    def _close(sock: Optional[socket.socket]) -> None:
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    # ------------------------------------------------------------------ #
    # Command channel (port 10001)
    # ------------------------------------------------------------------ #
    def _connect_command(self) -> None:
        sock = socket.create_connection(
            (self._ip, self._command_port), timeout=self._connect_timeout
        )
        sock.settimeout(self._command_timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._cmd_sock = sock
        self._cmd_buf = b""
        self._log(f"command channel connected to {self._ip}:{self._command_port}")

    @property
    def command_connected(self) -> bool:
        return self._cmd_sock is not None

    def send_command(
        self,
        command: Dict,
        raise_on_error: bool = True,
        timeout: Optional[float] = None,
    ) -> Dict:
        """Send a command dict and return the decoded acknowledgement dict.

        Serialized via a lock so only one request/response is in flight at a
        time, as required by the protocol.
        """
        payload = json.dumps(command, separators=(",", ":")).encode("utf-8")
        cmd_timeout = self._command_timeout if timeout is None else timeout
        with self._cmd_lock:
            if self._cmd_sock is None:
                self._connect_command()
            try:
                reply = self._send_locked(payload, cmd_timeout)
            except (OSError, ValueError, TimeoutError) as exc:
                # Drop the socket so the next call reconnects.
                self._log(f"command channel error ({exc}); reconnecting")
                self._close(self._cmd_sock)
                self._cmd_sock = None
                raise

        if raise_on_error:
            code = str(reply.get("errorCode", "0"))
            if code not in ("0", ""):
                raise LexiumError(
                    f"{command.get('cmdName')} failed: "
                    f"errorCode={code} errorMsg={reply.get('errorMsg', '')}"
                )
        return reply

    def _send_locked(self, payload: bytes, timeout: float) -> Dict:
        assert self._cmd_sock is not None
        self._cmd_sock.sendall(payload)
        deadline = time.monotonic() + timeout
        while True:
            obj = self._try_decode_cmd_buf()
            if obj is not None:
                return obj
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("timed out waiting for command acknowledgement")
            self._cmd_sock.settimeout(max(0.1, remaining))
            chunk = self._cmd_sock.recv(4096)
            if not chunk:
                raise ConnectionError("command channel closed by controller")
            self._cmd_buf += chunk

    def _try_decode_cmd_buf(self) -> Optional[Dict]:
        buf = self._cmd_buf.lstrip()
        if not buf:
            self._cmd_buf = buf
            return None
        try:
            obj, end = self._decoder.raw_decode(buf.decode("utf-8", "ignore"))
        except ValueError:
            return None
        # Recompute remaining bytes after the consumed object.
        consumed = buf[:end]
        self._cmd_buf = buf[len(consumed):]
        return obj

    # ------------------------------------------------------------------ #
    # Feedback channel (port 10000)
    # ------------------------------------------------------------------ #
    def _feedback_loop(self) -> None:
        decoder = json.JSONDecoder()
        while self._running:
            sock = None
            buf = b""
            try:
                sock = socket.create_connection(
                    (self._ip, self._feedback_port), timeout=self._connect_timeout
                )
                sock.settimeout(2.0)
                self._log(
                    f"feedback channel connected to {self._ip}:{self._feedback_port}"
                )
                while self._running:
                    try:
                        chunk = sock.recv(8192)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("feedback channel closed by controller")
                    buf += chunk
                    buf = self._drain_feedback(decoder, buf)
            except OSError as exc:
                self._log(f"feedback channel error ({exc}); retrying")
            finally:
                self._close(sock)
            if self._running:
                time.sleep(self._reconnect_backoff)

    def _drain_feedback(self, decoder: json.JSONDecoder, buf: bytes) -> bytes:
        text = buf.decode("utf-8", "ignore")
        text = text.lstrip()
        while text:
            try:
                obj, end = decoder.raw_decode(text)
            except ValueError:
                break
            with self._fb_lock:
                self._fb_state = obj
                self._fb_stamp = time.monotonic()
            text = text[end:].lstrip()
        return text.encode("utf-8")

    def get_feedback(self) -> Optional[Dict]:
        with self._fb_lock:
            return dict(self._fb_state) if self._fb_state is not None else None

    def feedback_age(self) -> float:
        """Seconds since the last feedback object (inf if none yet)."""
        with self._fb_lock:
            if self._fb_stamp == 0.0:
                return float("inf")
            return time.monotonic() - self._fb_stamp
