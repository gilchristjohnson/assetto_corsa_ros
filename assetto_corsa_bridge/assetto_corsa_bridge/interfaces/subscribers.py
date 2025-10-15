"""Subscribers that translate ROS commands into virtual controller inputs."""

from __future__ import annotations

import numpy as np
from autonoma_msgs.msg import VehicleInputs
import time

from assetto_corsa_bridge.utilities import VirtualRacingController

MAX_STEERING_DEG: float = 260.1
MAX_THROTTLE: float = 100.0
MAX_BRAKE: float = 6000.0


class Subscribers:
    """Mixin that handles ROS input topics and drives the virtual wheel."""

    def _init_subscribers(self) -> None:
        """Create subscriptions and initialise the backing virtual controller."""

        self.create_subscription(
            VehicleInputs, "/vehicle_inputs", self._vehicle_inputs_callback, 10
        )
        self._virtual_wheel = VirtualRacingController(
            name="Assetto Corsa Bridge Virtual Wheel"
        )
        self._assetto_gear : int = 0
        self._reported_gear : int = 0
        self._gear_buttons = tuple(f"GEAR_{idx}" for idx in range(1, 7))

    def _vehicle_inputs_callback(self, msg: VehicleInputs) -> None:
        """Map incoming ROS commands onto the virtual racing controller."""

        avg_brake_cmd = np.mean([msg.brake_f_cmd, msg.brake_r_cmd])

        steer_norm = float(np.clip(msg.steering_cmd / -MAX_STEERING_DEG, -1.0, 1.0))
        throttle_norm = float(np.clip(msg.throttle_cmd / MAX_THROTTLE, 0.0, 1.0))
        brake_norm = float(np.clip(avg_brake_cmd / MAX_BRAKE, 0.0, 1.0))

        axes = (
            ("STEERING", steer_norm, self._virtual_wheel.steer_max),
            ("THROTTLE", throttle_norm, self._virtual_wheel.pedal_max),
            ("BRAKE", brake_norm, self._virtual_wheel.pedal_max),
        )
        for axis, value, scale in axes:
            self._virtual_wheel.set_axis(axis, int(value * scale))

        commanded_gear = int(msg.gear_cmd)
        self._set_gear_button(commanded_gear)

    def _set_gear_button(self, commanded_gear: int) -> None:
        """Hold the button that corresponds to the requested gear."""

        if commanded_gear == self._assetto_gear :
            return

        for button in self._gear_buttons:
            self._virtual_wheel.release_button(button)
            
        self._virtual_wheel.press_button(self._gear_buttons[commanded_gear - 1])
        time.sleep(0.05)
        self._reported_gear = commanded_gear




