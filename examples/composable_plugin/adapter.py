"""Selected installed fake adapter factory. Replace only device-specific methods."""
def create_adapter(instance,context):
    from morla_runtime.adapters import FakeAdapter
    # Runtime already owns context['device_session'] resource locks. Creating an
    # adapter must not enable/home the device. FakeAdapter never opens hardware.
    return FakeAdapter(instance)
