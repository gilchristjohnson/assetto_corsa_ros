"""Subscribers that translate ROS commands into virtual controller inputs."""

from __future__ import annotations

import numpy as np
from time import monotonic

from autonoma_msgs.msg import VehicleInputs

from assetto_corsa_bridge.utilities import VirtualRacingController

MAX_STEERING_DEG: float = 260.1
MAX_THROTTLE: float = 100.0
MAX_BRAKE: float = 6000.0

GEAR_SHIFT_COOLDOWN_S: float = 0.25
GEAR_NEUTRAL_GRACE_S: float = 1.0


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
        self._desired_gear: int = 0
        self._last_non_neutral_gear: int = 0
        self._next_shift_allowed: float = 0.0
        self._neutral_grace_deadline: float = 0.0

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

        desired_gear = int(msg.gear_cmd)
        now = monotonic()

        if desired_gear != self._desired_gear:
            self._desired_gear = desired_gear
            self._neutral_grace_deadline = now + GEAR_NEUTRAL_GRACE_S
            self._next_shift_allowed = 0.0

        reported_gear = int(getattr(self, "_assetto_current_gear", self._last_gear))

        if reported_gear != 0:
            self._last_non_neutral_gear = reported_gear
        elif desired_gear != 0 and self._last_non_neutral_gear != 0 and now <= self._neutral_grace_deadline:
            reported_gear = self._last_non_neutral_gear

        if desired_gear == reported_gear:
            self._last_gear = reported_gear
            self._next_shift_allowed = 0.0
            return

        if reported_gear < 0 and desired_gear >= 0:
            if self._reverse_recovery_active:
                return
            self._reverse_recovery_active = True

        if now < self._next_shift_allowed:
            return

        shift_up = desired_gear > reported_gear
        self._virtual_wheel.tap_shift(up=shift_up, ms=50)

        self._next_shift_allowed = now + GEAR_SHIFT_COOLDOWN_S
        self._neutral_grace_deadline = now + GEAR_NEUTRAL_GRACE_S
