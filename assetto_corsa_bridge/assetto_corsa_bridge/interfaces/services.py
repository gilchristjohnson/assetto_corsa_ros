"""Services exposed by the Assetto Corsa bridge."""

from __future__ import annotations

from std_srvs.srv import Trigger


class Services:
    """Mixin that registers ROS service servers for the bridge node."""

    def _init_services(self) -> None:
        """Initialise ROS service endpoints for pausing and resetting the sim."""

        service_prefix = f"/{self.get_name()}"
        self.create_service(Trigger, f"{service_prefix}/pause", self._handle_pause_service)
        self.create_service(Trigger, f"{service_prefix}/reset", self._handle_reset_service)

    def _handle_pause_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Handle requests to pause the simulation via the virtual controller."""

        wheel = getattr(self, "_virtual_wheel", None)
        if wheel is None:
            response.success = False
            response.message = "Virtual racing controller not initialised."
            return response

        wheel.tap_pause()
        response.success = True
        response.message = "Pause button pressed."
        return response

    def _handle_reset_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Handle requests to reset the simulation via the virtual controller."""

        wheel = getattr(self, "_virtual_wheel", None)
        if wheel is None:
            response.success = False
            response.message = "Virtual racing controller not initialised."
            return response

        wheel.tap_reset()
        response.success = True
        response.message = "Reset button pressed."
        return response

