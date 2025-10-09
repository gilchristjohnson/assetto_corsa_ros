"""Helpers for decoding and working with Assetto Corsa telemetry packets."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Iterable, Iterator, Mapping, Protocol, runtime_checkable

try:  # pragma: no cover - import guard for different rclpy versions
    from rclpy.logging import Logger as _RclpyLogger
except ImportError:  # pragma: no cover - ROS 2 Humble no longer exposes Logger
    _RclpyLogger = None


@runtime_checkable
class Logger(Protocol):
    """Minimal logging protocol used by :class:`TelemetryDecoder`."""

    def warn(self, msg: str) -> Any:  # pragma: no cover - protocol definition
        ...

    def debug(self, msg: str) -> Any:  # pragma: no cover - protocol definition
        ...


if isinstance(_RclpyLogger, type):
    Logger.register(_RclpyLogger)

import numpy as np


@dataclass(frozen=True)
class TelemetryPacket:
    """Tiny wrapper with direct accessors for telemetry payload fields."""

    data: Mapping[str, Any]

    def scalar(self, key: str) -> float:
        return float(self.data[key])

    def float_sequence(self, key: str, length: int | None = None) -> np.ndarray:
        values = np.asarray(self.data[key], dtype=float)
        return values if length is None else values[:length]

    def contact_points(self) -> np.ndarray:
        raw_points = self.data["tire_contact_point"]
        if isinstance(raw_points, Mapping):
            items: Iterable[Any] = (raw_points[k] for k in sorted(raw_points))
        else:
            items = raw_points

        ordered = [np.asarray(value, dtype=float) for value in items]
        if not ordered:
            return np.empty((0, 3), dtype=float)

        array = np.asarray(ordered, dtype=float)
        return array.reshape(-1, array.shape[-1])

    def asdict(self) -> Mapping[str, Any]:
        return self.data


class TelemetryDecoder:
    """Decode UDP datagrams into :class:`TelemetryPacket` objects."""

    def __init__(self, _logger: Any) -> None:
        self._logger: Logger | None = _logger
        self._malformed_logged = False

    def _log_malformed(self, message: str, *, once: bool = True) -> None:
        logger = self._logger
        if logger is None:
            return

        if once:
            if not self._malformed_logged:
                logger.warn(message)
                self._malformed_logged = True
            else:
                logger.debug(message)
        else:
            logger.warn(message)

    def decode(self, datagram: bytes) -> Iterator[TelemetryPacket]:
        try:
            decoded = datagram.decode("utf-8")
        except UnicodeDecodeError as exc:
            self._log_malformed(f"Ignoring non UTF-8 telemetry datagram: {exc}")
            return

        for line in decoded.splitlines():
            stripped = line.strip()
            if not stripped:
                continue

            try:
                payload = json.loads(stripped)
            except json.JSONDecodeError as exc:
                self._log_malformed(
                    f"Ignoring malformed telemetry payload ({exc}) from datagram"
                )
                continue

            self._malformed_logged = False
            yield TelemetryPacket(payload)


__all__ = ["TelemetryDecoder", "TelemetryPacket"]
