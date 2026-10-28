"""CR5 component adapter; independent of all end-effector packages."""
import math
from morla_runtime.cached_actuator import CachedActuatorAdapter


class CR5Adapter(CachedActuatorAdapter):
    def validate_configuration(self):
        lo = self.parameters.get('joint_lower', [])
        hi = self.parameters.get('joint_upper', [])
        if len(lo) != 6 or len(hi) != 6 or any(
                not math.isfinite(a) or not math.isfinite(b) or a >= b
                for a, b in zip(lo, hi)):
            raise ValueError('CR5 requires six verified deployment joint_lower/joint_upper radians')

    def make_transport(self, parameters):
        from .transport import CR5Transport
        return CR5Transport(parameters)


def create_adapter(instance, context):
    if instance['backend'] == 'fake':
        from morla_runtime.adapters import FakeAdapter
        return FakeAdapter(instance)
    return CR5Adapter(instance, context)
