"""MORLA ABI: shared whole-robot session and no second command ingress."""
from .contract import PREFIX, role_of
from .session import DeviceSession


class Adapter:
    device_scoped_stop=True
    supports_watchdog=False # host watchdog cannot stop a disconnected controller

    def __init__(self, instance, session):
        self.instance=instance;self.key=instance['instance_id'];self.session=session
        self.active=False

    def configure(self, context):self.session.start()
    def read(self, now_ns):return self.session.snapshot(self.key)
    def health(self, now_ns):return self.read(now_ns)
    def prepare(self):
        if not self.health(0)['ready']:raise RuntimeError('MMK2_FEEDBACK_NOT_READY')
        return True
    def activate(self):
        self.session.activate(self.key);self.active=True;return True
    def submit(self, command):return self.session.submit(self.key,command)
    def release(self):return self.stop()
    def stop(self):
        self.active=False;return self.session.stop()
    def operate(self, operation):
        if operation=='prepare':return self.prepare()
        if operation=='enable':return self.activate() # software dispatch, not motor enable
        if operation=='stop':return self.stop()
        raise ValueError('MMK2_UNSUPPORTED_OPERATION: '+operation)
    def close(self):self.session.close()


def create_adapter(instance, context):
    device=context.get('device_session')
    members=device.get('instances',[]) if device else [instance['instance_id']]
    identity=device['session_id'] if device else instance['instance_id']
    rows={k:r for k,r in context['instances'].items() if k in members and r['plugin_id'].startswith(PREFIX)}
    if len(rows)!=len(members):raise ValueError('MMK2_SESSION_CONTAINS_FOREIGN_COMPONENT')
    shared=context['shared_device_sessions'];key=('mmk2_sdk',identity)
    if key not in shared:shared[key]=DeviceSession(rows,context['clock'],diagnostics=context.get('diagnostics'))
    if role_of(instance)=='camera':
        from .camera import CameraAdapter
        return CameraAdapter(instance,shared[key])
    return Adapter(instance,shared[key])
