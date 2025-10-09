"""Virtual controller helpers for driving Assetto Corsa."""

from __future__ import annotations

import time

from evdev import AbsInfo, UInput, ecodes


class VirtualRacingController:
    """Virtual USB racing wheel exposed via ``uinput``."""

    def __init__(self, name: str = "VirtualRacingController", bustype: int = 0x03) -> None:
        self._axes = {
            "STEERING": (ecodes.ABS_WHEEL, -32768, 32767, 32, 256, 0),
            "THROTTLE": (ecodes.ABS_GAS, 0, 1023, 0, 0, 0),
            "BRAKE": (ecodes.ABS_BRAKE, 0, 1023, 0, 0, 0),
        }
        self._buttons = {
            "SHIFT_UP": ecodes.BTN_GEAR_UP,
            "SHIFT_DOWN": ecodes.BTN_GEAR_DOWN,
            "PAUSE": ecodes.BTN_START,
            "RESET": ecodes.BTN_SELECT,
        }

        abs_caps = [
            (code, AbsInfo(0, minimum, maximum, fuzz, flat, resolution))
            for code, minimum, maximum, fuzz, flat, resolution in self._axes.values()
        ]
        capabilities = {ecodes.EV_KEY: list(self._buttons.values()), ecodes.EV_ABS: abs_caps}
        self._device = UInput(capabilities, name=name, bustype=bustype)

        _, self.steer_min, self.steer_max, *_ = self._axes["STEERING"]
        _, self.pedal_min, self.pedal_max, *_ = self._axes["THROTTLE"]

    def set_axis(self, axis: str, value: int) -> None:
        code, minimum, maximum, *_ = self._axes[axis.upper()]
        bounded = max(min(value, maximum), minimum)
        self._device.write(ecodes.EV_ABS, code, bounded)
        self._device.syn()

    def press_button(self, button: str) -> None:
        code = self._buttons[button.upper()]
        self._device.write(ecodes.EV_KEY, code, 1)
        self._device.syn()

    def release_button(self, button: str) -> None:
        code = self._buttons[button.upper()]
        self._device.write(ecodes.EV_KEY, code, 0)
        self._device.syn()

    def tap_button(self, button: str, ms: int = 40) -> None:
        self.press_button(button)
        time.sleep(ms / 1000.0)
        self.release_button(button)

    def press_shift(self, up: bool = True) -> None:
        self.press_button("SHIFT_UP" if up else "SHIFT_DOWN")

    def release_shift(self, up: bool = True) -> None:
        self.release_button("SHIFT_UP" if up else "SHIFT_DOWN")

    def tap_shift(self, up: bool = True, ms: int = 40) -> None:
        self.tap_button("SHIFT_UP" if up else "SHIFT_DOWN", ms)

    def tap_pause(self, ms: int = 40) -> None:
        self.tap_button("PAUSE", ms)

    def tap_reset(self, ms: int = 40) -> None:
        self.tap_button("RESET", ms)


__all__ = ["VirtualRacingController"]
