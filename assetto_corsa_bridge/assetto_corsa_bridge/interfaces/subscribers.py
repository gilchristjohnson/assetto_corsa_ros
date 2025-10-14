"""Subscribers that translate ROS commands into virtual controller inputs."""

from __future__ import annotations

import numpy as np
from typing import Optional

from autonoma_msgs.msg import VehicleInputs

from assetto_corsa_bridge.utilities import VirtualRacingController

MAX_STEERING_DEG: float = 260.1
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
        # reverse without waiting for a new gear command.  ``None`` means we
        # have not yet received any telemetry.
        self._assetto_current_gear: Optional[int] = None
        # Remember the highest forward gear requested while we exit reverse so
        # that we honour it after neutral is cleared.
        self._pending_forward_gear: Optional[int] = None

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

        current_gear = int(
            self._assetto_current_gear
            if self._assetto_current_gear is not None
            else self._last_gear
        )

        if commanded_gear <= 0:
            self._pending_forward_gear = None
        elif current_gear < 0:
            pending_target = self._pending_forward_gear or commanded_gear
            self._pending_forward_gear = max(pending_target, commanded_gear)
        elif (
            self._pending_forward_gear is not None
            and current_gear >= self._pending_forward_gear
        ):
            self._pending_forward_gear = None

        target_gear = (
            max(commanded_gear, self._pending_forward_gear)
            if self._pending_forward_gear is not None
            else commanded_gear
        )

        if target_gear == current_gear:
            self._last_gear = target_gear
            if target_gear >= 0:
                self._pending_forward_gear = None
            return

        shift_up = target_gear > current_gear
        step_count = abs(target_gear - current_gear)

        for _ in range(step_count):
            self._virtual_wheel.tap_shift(up=shift_up, ms=50)
            current_gear += 1 if shift_up else -1

        self._last_gear = target_gear
        if current_gear >= 0:
            self._pending_forward_gear = None
