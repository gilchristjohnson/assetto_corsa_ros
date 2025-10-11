"""Subscribers that translate ROS commands into virtual controller inputs."""

from __future__ import annotations

import numpy as np
from autonoma_msgs.msg import VehicleInputs

from assetto_corsa_bridge.utilities import VirtualRacingController

MAX_STEERING_DEG: float = 260
MAX_THROTTLE: float = 100.0
MAX_BRAKE: float = 6000.0


class Subscribers:
    """Mixin that handles ROS input topics and drives the virtual wheel."""

    def _init_subscribers(self) -> None:
        """Create subscriptions and initialise the backing virtual controller."""

        self.create_subscription(VehicleInputs, "/vehicle_inputs", self._vehicle_inputs_callback, 10)
        self._virtual_wheel = VirtualRacingController(name="Assetto Corsa Bridge Virtual Wheel")
        # Track the last commanded gear using Assetto Corsa numbering so we can
        # recover gracefully from neutral and reverse states.  Initialise to
        # ``0`` so the first command for gear ``1`` triggers an upshift.
        self._last_gear: int = 0
        # Cache the last reported Assetto Corsa gear so that we can recover from
        # reverse without waiting for a new gear command.
        self._assetto_current_gear: int = 0
        self._reverse_recovery_active: bool = False

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
        current_gear = int(getattr(self, "_assetto_current_gear", self._last_gear))
        needs_reverse_recovery = current_gear < 0 and commanded_gear >= 0

        if not needs_reverse_recovery and commanded_gear == self._last_gear:
            return

        if needs_reverse_recovery:
            if self._reverse_recovery_active:
                return
            self._reverse_recovery_active = True

        gear_difference = commanded_gear - current_gear

        if gear_difference == 0:
            self._last_gear = commanded_gear
            return

        shift_up = gear_difference > 0
        step_count = min(abs(gear_difference), 10)

        for _ in range(step_count):
            self._virtual_wheel.tap_shift(up=shift_up, ms=50)

        self._last_gear = commanded_gear
