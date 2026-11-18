"""Offline examples of the four external component families."""


def adapter(instance, context):
    from morla_runtime.adapters import FakeAdapter

    if instance['plugin_id'] != 'example.external.sensor':
        return FakeAdapter(instance)

    class Sensor(FakeAdapter):
        def read(self, now_ns):
            sample = super().read(now_ns)
            sample['ports']['telemetry'].pop('values')
            sample['ports']['telemetry']['fields'] = {
                'pressure': [[1., 2.], [3., 4.]], 'temperature': [21, 22],
            }
            return sample

    return Sensor(instance)


def kinematics(instance):
    """No-motion solver that exercises plugin discovery in fake mode."""
    import numpy as np

    class Solver:
        def sync_state(self, q):
            self.seed = np.asarray(q, dtype=float)

        def fk(self, q):
            return np.eye(4)

        def solve(self, target):
            return self.seed.copy()

    return Solver()
