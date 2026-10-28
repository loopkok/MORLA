"""ROS transport owns no authority: all ingress calls the same runtime gateway.

No direct public driver topics are published here. Real deployment adapters must
own session-private driver endpoints and provide verified stop/watchdog behavior.
"""
import json
from pathlib import Path
from .session import RuntimeSession


def main(args=None, *, node_name='morla_runtime', required_plugin=None,
         publish_joint_states=False):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from morla_interfaces.msg import CommandEnvelope, CommandReceipt, ComponentHealth
    from morla_interfaces.srv import RuntimeOperation, ControlAuthority

    rclpy.init(args=args)

    class RuntimeNode(Node):
        def __init__(self):
            super().__init__(node_name)
            self.declare_parameter('resolved_path', '')
            path = self.get_parameter('resolved_path').value
            resolved = json.loads(Path(path).read_text())
            if required_plugin is not None:
                instances = resolved.get('instances', [])
                if len(instances) != 1 or instances[0]['plugin_id'] != required_plugin:
                    raise ValueError('Single-device driver requires exactly its own component')
            self.runtime = RuntimeSession(resolved)
            self.joint_publishers = {}
            self.last_samples = {}
            if publish_joint_states:
                from sensor_msgs.msg import JointState
                self.joint_publishers = {
                    key: self.create_publisher(JointState, '~/' + key + '/joint_states', 10)
                    for key, instance in self.runtime.instances.items()
                    if instance['kind'] == 'actuator'
                }
            self.state_pub = self.create_publisher(String, '~/state', 10)
            self.receipts = self.create_publisher(CommandReceipt, '~/receipts', 10)
            self.health_pub = self.create_publisher(ComponentHealth, '~/health', 10)
            self.create_subscription(CommandEnvelope, '~/commands', self.command, 1)
            self.create_service(RuntimeOperation, '~/operation', self.operation)
            self.create_service(ControlAuthority, '~/authority', self.authority)
            self.runtime.start()
            self.create_timer(0.02, self.tick)

        def command(self, msg):
            payload = {key: getattr(msg, key) for key in (
                'session_id', 'producer', 'token', 'epoch', 'sequence', 'issued_ns', 'ttl_ns')}
            payload['commands'] = [{key: list(getattr(item, key)) if key in ('names', 'values', 'units') else getattr(item, key)
                                    for key in ('component_id', 'mode', 'names', 'values', 'units')} for item in msg.commands]
            result = self.runtime.submit(payload)
            reply = CommandReceipt()
            reply.session_id, reply.sequence = self.runtime.session_id, msg.sequence
            reply.accepted, reply.reason = result['accepted'], result.get('reason', '')
            reply.dispatched = result.get('dispatched', [])
            reply.monotonic_ns = self.runtime.clock()
            self.receipts.publish(reply)

        def operation(self, req, reply):
            reply.request_id = req.request_id
            try:
                if req.session_id != self.runtime.session_id:
                    raise ValueError('SESSION_MISMATCH')
                result = self.runtime.invoke(req.operation, req.request_id, list(req.component_ids) or None)
                reply.status, reply.error = result['status'], result.get('error', '')
            except Exception as exc:
                reply.status, reply.error = 'FAILED', str(exc)
            return reply

        def authority(self, req, reply):
            try:
                if req.session_id != self.runtime.session_id:
                    raise ValueError('SESSION_MISMATCH')
                if req.operation == 'acquire':
                    result = self.runtime.acquire(req.producer, list(req.resources) or None, req.ttl_ns, req.request_id)
                elif req.operation == 'renew':
                    result = self.runtime.renew(req.producer, req.token, req.ttl_ns)
                elif req.operation == 'revoke':
                    self.runtime.revoke(req.producer)
                    result = {}
                else:
                    raise ValueError('UNSUPPORTED_AUTHORITY_OPERATION')
                reply.accepted = True
                reply.token, reply.epoch = result.get('token', ''), result.get('epoch', self.runtime.epoch)
                reply.expires_ns = result.get('expires_ns', 0)
            except Exception as exc:
                reply.accepted, reply.reason = False, str(exc)
            return reply

        def tick(self):
            state = self.runtime.tick()
            message = String()
            message.data = json.dumps(state)
            self.state_pub.publish(message)
            for key, value in state['components'].items():
                health = ComponentHealth()
                health.session_id, health.component_id = self.runtime.session_id, key
                for field in ('ready', 'fresh', 'measured', 'synthetic'):
                    setattr(health, field, bool(value.get(field, False)))
                health.source, health.sample_ns = value.get('source', 'unknown'), value.get('sample_ns', 0)
                health.values = value.get('values', [])
                self.health_pub.publish(health)
                if key in self.joint_publishers and value.get('ready') and value.get('values'):
                    stamp = value['sample_ns']
                    if stamp <= self.last_samples.get(key, -1):
                        continue
                    from sensor_msgs.msg import JointState
                    message = JointState()
                    ros_stamp = self.get_clock().now().nanoseconds - (self.runtime.clock() - stamp)
                    message.header.stamp.sec = ros_stamp // 1_000_000_000
                    message.header.stamp.nanosec = ros_stamp % 1_000_000_000
                    message.header.frame_id = key + '_base'
                    message.name = [joint['name'] for joint in self.runtime.instances[key]['joints']]
                    message.position = value['values']
                    self.joint_publishers[key].publish(message)
                    self.last_samples[key] = stamp

    node = None
    try:
        node = RuntimeNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.runtime.stop()
            node.destroy_node()
        rclpy.shutdown()


def joint_state_main(args=None):
    """Generic multi-component gateway with named JointState feedback outputs."""
    return main(args, publish_joint_states=True)
