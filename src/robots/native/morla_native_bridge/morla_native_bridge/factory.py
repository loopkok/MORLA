"""One factory per logical component, one connection per physical session."""
from types import SimpleNamespace
from .native_ros import NativeDeviceSession,NativeRosAdapter


def create_adapter(instance,context):
    device=context['device_session']
    if device is None:raise ValueError('native component requires device session')
    key=('morla.native_bridge',device['session_id'])
    shared=context['shared_device_sessions']
    if key not in shared:
        host=SimpleNamespace(session_id=context['session_id'],clock=context['clock'],
            resources=SimpleNamespace(lane_fd=context['lane_fd'],root=context['resource_root']))
        rows=list(context['instances'].values())
        shared[key]=NativeDeviceSession(device,rows,host)
    return NativeRosAdapter(instance,shared[key])
