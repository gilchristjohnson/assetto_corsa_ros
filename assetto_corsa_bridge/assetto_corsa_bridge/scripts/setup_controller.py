"""Helper to map VirtualRacingController in Assetto Corsa (Content Manager).

This guide pauses at each navigation step so you can get to the correct page
*before* any bindings begin. There is no countdown—only a slight delay when
engaging an input so CM can capture it cleanly.

Important: after you click **Click to assign** in CM, click back into this terminal
window before pressing Enter to start the binding.
"""

from __future__ import annotations

import time
import argparse
from dataclasses import dataclass
from typing import Callable, Iterable

from ..utilities import VirtualRacingController


def _line(char: str = "─", width: int = 60) -> str:
    return char * width

def _title(text: str) -> None:
    print("\n" + _line())
    print(text)
    print(_line() + "\n")

def _gate(message: str, prompt: str = "Press Enter here when done...") -> None:
    print(message)
    input(prompt + "\n")


@dataclass(frozen=True)
class _Step:
    description: str
    engage: Callable[[], None]
    release: Callable[[], None]
    prompt: str = "Press Enter to release and continue..."


def _run_steps(steps: Iterable[_Step], delay_before_engage: float) -> None:
    for step in steps:
        print(_line())
        print(step.description)
        # Explicit instruction to click back into terminal before starting
        input("Click back into this terminal, then press Enter to start this binding...")
        if delay_before_engage > 0:
            time.sleep(delay_before_engage)
        step.engage()
        try:
            input(step.prompt)
        finally:
            step.release()
    print("\n" + _line() + "\nMapping complete.\n" + _line())


def main() -> None:
    parser = argparse.ArgumentParser(description="Map VirtualRacingController to Assetto Corsa (Content Manager).")
    parser.add_argument("--delay", type=float, default=0.6, help="Seconds to wait after pressing Enter before engaging (no countdown).")
    args = parser.parse_args()
    DELAY = max(0.0, float(args.delay))

    controller = VirtualRacingController()
    steering_neutral = (controller.steer_min + controller.steer_max) // 2
    pedal_neutral = controller.pedal_min

    _title("Navigation")
    _gate("Open Content Manager, then navigate to the following page\n**SETTINGS** -> **ASSETTO CORSA** -> **CONTROLS**")
    _gate("Set **Input method = Wheel**.")

    _title("Axes")
    _gate("Open the **AXIS** tab in CONTROLS. ", prompt="Press Enter here when the Axes tab is open...")
    axis_steps = [
        _Step(
            description="In CM, click **Click to assign** for Steering.",
            engage=lambda: controller.set_axis("STEERING", controller.steer_max),
            release=lambda: controller.set_axis("STEERING", steering_neutral),
        ),
        _Step(
            description="In CM, click **Click to assign** for Throttle (Gas).",
            engage=lambda: controller.set_axis("THROTTLE", controller.pedal_max),
            release=lambda: controller.set_axis("THROTTLE", pedal_neutral),
        ),
        _Step(
            description="In CM, click **Click to assign** for Brakes.",
            engage=lambda: controller.set_axis("BRAKE", controller.pedal_max),
            release=lambda: controller.set_axis("BRAKE", pedal_neutral),
        ),
    ]
    _run_steps(axis_steps, DELAY)

    _title("Buttons")
    _gate("Switch to the **BUTTONS** tab.", prompt="Press Enter here when the **BUTTONS** tab is open...")
    button_steps = [
        _Step(
            description="In CM, click **Click to assign** for **Next gear**.",
            engage=lambda: controller.press_button("SHIFT_UP"),
            release=lambda: controller.release_button("SHIFT_UP"),
        ),
        _Step(
            description="In CM, click **Click to assign** for **Previous gear**.",
            engage=lambda: controller.press_button("SHIFT_DOWN"),
            release=lambda: controller.release_button("SHIFT_DOWN"),
        ),
    ]
    _run_steps(button_steps, DELAY)

    _title("System")
    _gate("Switch to the **SYSTEM** tab.\nNote: Use the **Click to assign** buttons with the **No modifier +** messages.", prompt="Press Enter here when the System tab is open...")
    system_steps = [
        _Step(
            description="In CM, click **Click to assign** for **Pause race**.",
            engage=lambda: controller.press_button("PAUSE"),
            release=lambda: controller.release_button("PAUSE"),
        ),
        _Step(
            description="In CM, click **Click to assign** for **Teleport to Pits**.",
            engage=lambda: controller.press_button("RESET"),
            release=lambda: controller.release_button("RESET"),
        ),
    ]
    _run_steps(system_steps, DELAY)

    _title("Fine-tune Settings")
    _gate(
        "Still in Controls, open **AXIS** to fine-tune:\n"
        " • Set all **Gamma** to **1.00**\n"
        " • Set *Degrees* for *Steering* to **260**\n"
        "When finished, **save your preset** in Content Manager.",
        prompt="Press Enter here when you’ve finished tweaking settings..."
    )

    _gate("Navigate to **DRIVE**, then click on the **...** near **Tyre blankets**")
    _gate(
        " • Uncheck **Automatic Shifting**\n"
        " • Check **Automatic Clutch**\n"
        " • Set Traction Control to **Off**\n"
        " • Set ABS to **Off**\n"
        "When finished, **save your preset** in Content Manager.",
        prompt="Press Enter here when you’ve finished tweaking settings..."
    )

    try:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted. Releasing inputs...")
    finally:
        controller.set_axis("STEERING", steering_neutral)
        controller.set_axis("THROTTLE", pedal_neutral)
        controller.set_axis("BRAKE", pedal_neutral)
        for btn in ("SHIFT_UP", "SHIFT_DOWN", "PAUSE", "RESET"):
            controller.release_button(btn)
        print("\n" + _line() + "\nAll inputs released. You can close this window.\n" + _line())


if __name__ == "__main__":
    main()
