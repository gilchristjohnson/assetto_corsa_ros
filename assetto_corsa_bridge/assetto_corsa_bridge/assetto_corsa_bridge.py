"""ROS 2 node that bridges Assetto Corsa telemetry and commands."""

from __future__ import annotations

import logging
import time

import rclpy
from rclpy.context import Context
from rclpy.node import Node

from .interfaces import Publishers, Services, Subscribers
from .utilities.socket import AssettoSocketManager
from .utilities.telemetry import TelemetryDecoder

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
        self._socket_manager = AssettoSocketManager(
            self.get_logger(), self._assetto_host, self._assetto_port
        )
        self._socket_manager.ensure_socket()
        self.create_timer(1.0 / poll_hz, self._poll_assetto)

    def _poll_assetto(self) -> None:
        """Drain the UDP socket and publish ROS messages for each telemetry packet."""

        if not self._socket_manager.ensure_socket():
            return

        try:
            for datagram, addr in self._socket_manager.iterate_payloads():
                stamp = self.get_clock().now().to_msg()
                handled = False

                for packet in self._decoder.decode(datagram):
                    self._publish_assetto_packet(stamp, packet)
                    handled = True

                self._socket_manager.send_ack(addr, handled)
        except OSError as exc:
            self._socket_manager.record_error(
                "Error while polling Assetto Corsa socket", exc
            )

    def destroy_node(self) -> None:
        """Ensure sockets are closed before the node is destroyed."""

        self._socket_manager.close()
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
