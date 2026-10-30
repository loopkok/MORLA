"""Aero Hand Open component adapter; independent of all arm packages."""
from morla_runtime.cached_actuator import CachedActuatorAdapter


class LoopHandAdapter(CachedActuatorAdapter):
    feedback_source = 'estimated'
    feedback_measured = False
    # Upstream firmware has no acknowledged stop or verified communication watchdog.
    supports_watchdog = False
    activation_error = 'LoopHand real is read-only: stop/watchdog unverified'

    def activate(self):
        # Keep real motion closed regardless of deployment flags: the firmware
        # has no acknowledged stop and no verified communication watchdog.
        raise RuntimeError(self.activation_error)

    def make_transport(self, parameters):
        from .transport import LoopHandTransport
        return LoopHandTransport(parameters)


def create_adapter(instance, context):
    if instance['backend'] == 'fake':
        from morla_runtime.adapters import FakeAdapter
        return FakeAdapter(instance)
    return LoopHandAdapter(instance, context)
