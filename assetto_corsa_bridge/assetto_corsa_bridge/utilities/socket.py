"""Resilient UDP socket helpers for the Assetto Corsa bridge."""

from __future__ import annotations

import json
import socket
import time
from collections.abc import Iterator
from typing import Tuple

_Address = Tuple[str, int]


class AssettoSocketManager:
    """Manage the UDP socket lifecycle for the Assetto Corsa bridge."""

    def __init__(
        self,
        logger,
        host: str,
        port: int,
        *,
        error_log_interval: float = 5.0,
    ) -> None:
        self._logger = logger
        self._host = host
        self._port = port
        self._error_log_interval = error_log_interval

        self._sock: socket.socket | None = None
        self._socket_error_logged_at: float | None = None
        self._had_socket_error = False

    # ---------------------------------------------------------------------
    # Public API
    # ---------------------------------------------------------------------
    def ensure_socket(self) -> bool:
        """Ensure the UDP socket is ready for use."""

        if self._sock is not None:
            return True

        try:
            self._sock = self._create_socket()
        except OSError as exc:
            self.record_error("Failed to create Assetto Corsa socket", exc)
            return False

        if self._had_socket_error:
            self._logger.info("Successfully recreated Assetto Corsa socket")
            self._had_socket_error = False

        return True

    def iterate_payloads(self) -> Iterator[tuple[bytes, _Address]]:
        """Yield incoming UDP datagrams until the socket is drained."""

        if not self.ensure_socket():
            return

        sock = self._sock
        if sock is None:
            return

        while True:
            try:
                yield sock.recvfrom(65536)
            except BlockingIOError:
                return
            except InterruptedError:
                continue
            except OSError as exc:
                self.record_error("Error receiving Assetto Corsa telemetry", exc)
                return

    def send_ack(self, addr: _Address, handled: bool) -> None:
        """Send a JSON acknowledgement back to the Assetto sender."""

        sock = self._sock
        if sock is None:
            return

        payload = json.dumps({"bridge": "ack", "handled": handled}).encode("utf-8")

        try:
            sock.sendto(payload, addr)
        except OSError as exc:
            self.record_error("Error sending Assetto Corsa acknowledgement", exc)

    def record_error(self, context: str, exc: OSError) -> None:
        """Record a socket error, throttling logs and closing the socket."""

        self._had_socket_error = True
        now = time.monotonic()
        if (
            self._socket_error_logged_at is None
            or now - self._socket_error_logged_at > self._error_log_interval
        ):
            self._socket_error_logged_at = now
            self._logger.error("%s: %s", context, exc)
        else:
            self._logger.debug("%s: %s", context, exc)

        self.close()

    def close(self) -> None:
        """Close the socket if it exists."""

        sock, self._sock = self._sock, None
        if sock is None:
            return

        sock.close()

    # ---------------------------------------------------------------------
    # Internal helpers
    # ---------------------------------------------------------------------
    def _create_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._host, self._port))
        sock.setblocking(False)
        self._logger.info("Assetto UDP listening on %s:%s", self._host, self._port)
        return sock

