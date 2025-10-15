"""Socket helpers for the Assetto Corsa bridge."""

from __future__ import annotations

from collections.abc import Iterator
import json
import socket
from typing import Final, Protocol

_BUFFER_SIZE: Final[int] = 65536


_Address = tuple[str, int]


class _Logger(Protocol):
    """Subset of the ``rclpy`` logger API used by the socket helper."""

    def info(self, msg: str, *args: object) -> None:  # pragma: no cover - protocol
        """Log an informational message."""

    def warning(self, msg: str, *args: object) -> None:  # pragma: no cover - protocol
        """Log a warning message."""


class AssettoSocketError(RuntimeError):
    """Raised when the Assetto bridge encounters a socket failure."""


class AssettoSocket:
    """Non-blocking UDP helper tailored for the Assetto Corsa bridge."""

    def __init__(self, host: str, port: int, logger: _Logger) -> None:
        self._address: _Address = (host, port)
        self._logger = logger
        self._sock = self._open()

    def datagrams(self) -> Iterator[tuple[bytes, _Address]]:
        """Yield any pending datagrams, restarting the bridge on errors."""

        sock = self._sock
        if sock is None:
            return

        while True:
            try:
                yield sock.recvfrom(_BUFFER_SIZE)
            except BlockingIOError:
                return
            except InterruptedError:
                continue
            except OSError as exc:  # pragma: no cover - network failure
                self._fail("receiving telemetry", exc)

    def send_ack(self, addr: _Address, handled: bool) -> None:
        """Send an acknowledgement for the processed datagram."""

        sock = self._sock
        if sock is None:
            return

        response = json.dumps({"bridge": "ack", "handled": handled}).encode("utf-8")
        try:
            sock.sendto(response, addr)
        except OSError as exc:  # pragma: no cover - network failure
            self._fail("sending acknowledgement", exc)

    def close(self) -> None:
        """Close the socket aggressively, ignoring shutdown errors."""

        sock, self._sock = self._sock, None
        if sock is None:
            return

        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

        sock.close()

    def _open(self) -> socket.socket:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(self._address)
            sock.setblocking(False)
        except OSError as exc:
            host, port = self._address
            self._logger.warning(
                "Failed to bind Assetto UDP socket on %s:%s; restarting bridge: %s",
                host,
                port,
                exc,
            )
            raise AssettoSocketError("Unable to bind Assetto UDP socket") from exc

        host, port = self._address
        self._logger.info("Assetto UDP listening on %s:%s", host, port)
        return sock

    def _fail(self, action: str, exc: OSError) -> None:
        self._logger.warning(
            "Socket error while %s; restarting bridge: %s",
            action,
            exc,
        )
        self.close()
        raise AssettoSocketError(f"Socket error while {action}") from exc


__all__ = ["AssettoSocket", "AssettoSocketError"]
