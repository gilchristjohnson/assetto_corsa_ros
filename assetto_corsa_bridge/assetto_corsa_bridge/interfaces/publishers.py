"""Publishers that expose Assetto Corsa telemetry via ROS topics."""

from __future__ import annotations

from autonoma_msgs.msg import PowertrainData, VehicleData
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

import numpy as np

from assetto_corsa_bridge.utilities.frames import FrameTransformer
from assetto_corsa_bridge.utilities.telemetry import TelemetryPacket


class Publishers:
    """Mixin that sets up ROS publishers for Assetto Corsa telemetry."""

    def _init_publishers(self):
        """Create ROS publishers and cache coordinate transform helpers."""

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/state/odom", 10)
        self.vehicle_data_pub = self.create_publisher(VehicleData, "/vehicle_data", 10)
        self.powertrain_pub = self.create_publisher(PowertrainData, "/powertrain_data", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._frame = FrameTransformer()

    def _publish_clock(self):
        """Publish the current ROS clock reading."""

        msg = Clock()
        msg.clock = self.get_clock().now().to_msg()
        self.clock_pub.publish(msg)

    def _publish_transform(
        self, stamp: Time, position_map: np.ndarray, orientation: np.ndarray
    ) -> None:
        """Broadcast the transform from ``map`` to ``base_link``."""

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = position_map.tolist()

        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = orientation.tolist()

        self.tf_broadcaster.sendTransform(transform)

    def _publish_odometry(
        self,
        stamp: Time,
        position_map: np.ndarray,
        orientation: np.ndarray,
        linear_velocity: np.ndarray,
        angular_velocity: np.ndarray,
    ) -> None:
        """Publish vehicle pose in ``map`` frame and twist in ``base_link``."""

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position_map.tolist()

        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = orientation.tolist()

        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = linear_velocity.tolist()

        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = angular_velocity.tolist()

        self.odom_pub.publish(odom)

    def _publish_vehicle_data(self, stamp: Time, packet: TelemetryPacket):
        """Publish wheel speeds, steering angle, loads, temperatures, and brake pressure."""

        vehicle = VehicleData()
        vehicle.header.stamp = stamp

        wheel_speed = packet.float_sequence("wheel_angular_speed", length=4)
        vehicle.ws_front_left, vehicle.ws_front_right, vehicle.ws_rear_left, vehicle.ws_rear_right = wheel_speed.tolist()

        steer = packet.scalar("steer")
        vehicle.steering_wheel_angle = steer
        vehicle.steering_wheel_angle_cmd = steer

        wheel_load = packet.float_sequence("load", length=4)
        vehicle.fl_wheel_load, vehicle.fr_wheel_load, vehicle.rl_wheel_load, vehicle.rr_wheel_load = wheel_load.tolist()

        tire_temp = packet.float_sequence("current_tyres_core_temp", length=4)
        vehicle.fl_tire_temperature, vehicle.fr_tire_temperature, vehicle.rl_tire_temperature, vehicle.rr_tire_temperature = tire_temp.tolist()

        brake = packet.scalar("brake")
        vehicle.front_brake_pressure = brake
        vehicle.rear_brake_pressure = brake

        self.vehicle_data_pub.publish(vehicle)

    def _publish_powertrain_data(self, stamp: Time, packet: TelemetryPacket):
        """Publish engine speed, throttle position, and vehicle speed."""

        powertrain = PowertrainData()
        powertrain.header.stamp = stamp
        powertrain.engine_rpm = packet.scalar("rpm")
        powertrain.throttle_position = packet.scalar("gas")
        powertrain.vehicle_speed_kmph = packet.scalar("speed_ms") * 3.6

        assetto_gear = int(packet.scalar("gear"))
        self._assetto_current_gear = assetto_gear
        if assetto_gear >= 0:
            self._reverse_recovery_active = False

        powertrain.current_gear = int(getattr(self, "_last_gear", assetto_gear))

        self.powertrain_pub.publish(powertrain)

    def _publish_assetto_packet(self, stamp: Time, packet: TelemetryPacket):
        """Publish the complete set of ROS messages derived from a telemetry packet."""

        orientation = self._frame.orientation_from_contact_points(packet.contact_points())
        position_map = self._frame.world_to_map(
            packet.float_sequence("world_position", length=3)
        )
        linear_velocity = self._frame.assetto_local_to_base(
            packet.float_sequence("local_velocity", length=3)
        )
        angular_velocity = self._frame.assetto_local_to_base(
            packet.float_sequence("local_angular_velocity", length=3)
        )

        self._publish_vehicle_data(stamp, packet)
        self._publish_powertrain_data(stamp, packet)
        self._publish_transform(stamp, position_map, orientation)
        self._publish_odometry(stamp, position_map, orientation, linear_velocity, angular_velocity)
