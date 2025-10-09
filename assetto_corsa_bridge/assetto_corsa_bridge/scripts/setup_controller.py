"""Interactive helper for exercising the virtual racing controller inputs."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import Callable, Iterable

from ..utilities import VirtualRacingController


@dataclass(frozen=True)
class _Step:
    """Single controller action for the setup walkthrough."""
    description: str
    engage: Callable[[], None]
    release: Callable[[], None]
    prompt: str = "Press Enter to release and continue..."
    countdown_seconds: int = 3  # seconds to count down before engage


def _countdown(seconds: int) -> None:
    """Show a simple countdown inline."""
    if seconds <= 0:
        return
    print("Starting in: ", end="", flush=True)
    for s in range(seconds, 0, -1):
        sys.stdout.write(f"{s}")
        sys.stdout.flush()
        time.sleep(1)
        if s > 1:
            sys.stdout.write(" ")
            sys.stdout.flush()
    print()  # newline after countdown


def _run_steps(steps: Iterable[_Step]) -> None:
    """Iterate through the provided controller actions."""
    for step in steps:
        print(f"\n{step.description}")
        # Countdown (if any) happens BEFORE the input is engaged
        _countdown(step.countdown_seconds)

        step.engage()
        try:
            input(step.prompt)
        finally:
            step.release()

    print("\nController input walkthrough complete.")


def main() -> None:
    """Guide the user through each of the controller inputs."""

    controller = VirtualRacingController()

    steering_neutral = (controller.steer_min + controller.steer_max) // 2
    pedal_neutral = controller.pedal_min

    # Default countdown for active steps
    COUNTDOWN = 3

    steps = [
        # Gate: wait for the user to start (no countdown here)
        _Step(
            description="Ready to begin",
            engage=lambda: None,
            release=lambda: None,
            prompt="Press Enter to start...",
            countdown_seconds=0,
        ),

        _Step(
            description="Steering max",
            engage=lambda: controller.set_axis("STEERING", controller.steer_max),
            release=lambda: controller.set_axis("STEERING", steering_neutral),
            countdown_seconds=COUNTDOWN,
        ),
        _Step(
            description="Throttle max",
            engage=lambda: controller.set_axis("THROTTLE", controller.pedal_max),
            release=lambda: controller.set_axis("THROTTLE", pedal_neutral),
            countdown_seconds=COUNTDOWN,
        ),
        _Step(
            description="Brake max",
            engage=lambda: controller.set_axis("BRAKE", controller.pedal_max),
            release=lambda: controller.set_axis("BRAKE", pedal_neutral),
            countdown_seconds=COUNTDOWN,
        ),
        _Step(
            description="Gear up",
            engage=lambda: controller.press_button("SHIFT_UP"),
            release=lambda: controller.release_button("SHIFT_UP"),
            countdown_seconds=COUNTDOWN,
        ),
        _Step(
            description="Gear down",
            engage=lambda: controller.press_button("SHIFT_DOWN"),
            release=lambda: controller.release_button("SHIFT_DOWN"),
            countdown_seconds=COUNTDOWN,
        ),
        _Step(
            description="Pause",
            engage=lambda: controller.press_button("PAUSE"),
            release=lambda: controller.release_button("PAUSE"),
            countdown_seconds=COUNTDOWN,
        ),
        _Step(
            description="Reset",
            engage=lambda: controller.press_button("RESET"),
            release=lambda: controller.release_button("RESET"),
            countdown_seconds=COUNTDOWN,
        ),
    ]

    print(
        "Starting controller setup walkthrough. Each input will engage after a brief countdown "
        "and remain active until you press Enter."
    )

    try:
        _run_steps(steps)
    except KeyboardInterrupt:
        print("\nSetup interrupted. Releasing any active inputs...")
    finally:
        controller.set_axis("STEERING", steering_neutral)
        controller.set_axis("THROTTLE", pedal_neutral)
        controller.set_axis("BRAKE", pedal_neutral)
        for button in ("SHIFT_UP", "SHIFT_DOWN", "PAUSE", "RESET"):
            controller.release_button(button)

        print("All controller inputs released.")


if __name__ == "__main__":  # pragma: no cover - manual invocation helper
    main()