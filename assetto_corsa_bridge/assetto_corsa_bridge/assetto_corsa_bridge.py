"""ROS 2 node that bridges Assetto Corsa telemetry and commands."""

from __future__ import annotations

import json
import logging
import socket
import time
from typing import Iterator

import rclpy
from rclpy.context import Context
from rclpy.node import Node

from .interfaces import Publishers, Services, Subscribers
from .utilities.telemetry import TelemetryDecoder

_Address = tuple[str, int]

_LOGGER = logging.getLogger(__name__)

class AssettoCorsaBridge(Node, Publishers, Subscribers, Services):
    """ROS 2 node that proxies UDP telemetry into ROS messages and commands back."""

    def __init__(self, *, context: Context | None = None) -> None:
        super().__init__("assetto_corsa_bridge", context=context)

        self.dt = float(self.declare_parameter("dt", 1.0 / 100.0).value)
        poll_hz = float(self.declare_parameter("assetto_poll_hz", 200.0).value)
        self.get_logger().info(f"Simulation time step: {self.dt} seconds")

        self._assetto_host = self.declare_parameter("assetto_host", "127.0.0.1").value
        self._assetto_port = int(self.declare_parameter("assetto_port", 15000).value)

        self.declare_parameter("lat0", -252.99)
        self.declare_parameter("lon0", -298.13)
        self.declare_parameter("alt0", -11.89)

        self._init_subscribers()
        self._init_publishers()
        self._init_services()
        self.create_timer(self.dt, self._publish_clock)

        self._decoder = TelemetryDecoder(self.get_logger())
        self._sock: socket.socket | None = None

        self._ensure_socket()
        self.create_timer(1.0 / poll_hz, self._poll_assetto)

    def _create_assetto_socket(self) -> socket.socket:
        """Create and bind the UDP socket used to receive telemetry."""

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._assetto_host, self._assetto_port))
        sock.setblocking(False)
        self.get_logger().info(
            f"Assetto UDP listening on {self._assetto_host}:{self._assetto_port}"
        )
        return sock

    def _poll_assetto(self) -> None:
        """Drain the UDP socket and publish ROS messages for each telemetry packet."""

        if not self._ensure_socket() or self._sock is None:
            return

        for datagram, addr in self._read_socket_payloads(self._sock):
            stamp = self.get_clock().now().to_msg()
            handled = False

            for packet in self._decoder.decode(datagram):
                self._publish_assetto_packet(stamp, packet)
                handled = True

            self._send_ack(addr, handled)

    def _read_socket_payloads(self, sock: socket.socket) -> Iterator[tuple[bytes, _Address]]:
        """Yield raw UDP datagrams until the socket has no more payloads."""

        while True:
            try:
                yield sock.recvfrom(65536)
            except BlockingIOError:
                return
            except InterruptedError:
                continue

    def _send_ack(self, addr: _Address, handled: bool) -> None:
        """Respond to the sender confirming whether the payload produced output."""

        sock = self._sock
        if sock is None:
            return

        response: dict[str, object] = {"bridge": "ack", "handled": handled}

        payload = json.dumps(response).encode("utf-8")
        sock.sendto(payload, addr)

    def _ensure_socket(self) -> bool:
        """Make sure the UDP socket exists, logging failures once."""

        if self._sock is not None:
            return True

        self._sock = self._create_assetto_socket()
        return True

    def _close_socket(self) -> None:
        """Close the existing socket and clear internal state."""

        sock, self._sock = self._sock, None
        if sock is None:
            return

        sock.close()

    def destroy_node(self) -> None:
        """Ensure sockets are closed before the node is destroyed."""

        self._close_socket()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Entry point that spins the Assetto Corsa bridge node."""

    rclpy.init(args=args)

    restart_delay = 1.0
    max_restart_delay = 30.0

    try:
        while rclpy.ok():
            node: AssettoCorsaBridge | None = None

            try:
                node = AssettoCorsaBridge()
            except Exception:  # pragma: no cover - defensive startup retry
                _LOGGER.exception("Failed to initialize Assetto Corsa bridge; retrying")
                if not rclpy.ok():
                    break

                time.sleep(restart_delay)
                restart_delay = min(restart_delay * 2, max_restart_delay)
                continue

            should_restart = False

            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                node.get_logger().info("Shutdown requested. Exiting.")
                break
            except Exception:
                node.get_logger().exception(
                    "Assetto Corsa bridge crashed unexpectedly; restarting"
                )
                should_restart = True
            finally:
                node.destroy_node()

            if not should_restart:
                break

            if not rclpy.ok():
                break

            time.sleep(restart_delay)
            restart_delay = min(restart_delay * 2, max_restart_delay)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
