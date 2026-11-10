"""ROS transport owns no authority: all ingress calls the same runtime gateway.

No direct public driver topics are published here. Device adapters own their
transport endpoints and report actual feedback/stop outcomes.
"""
import json
from pathlib import Path
from .session import RuntimeSession


def has_si_joint_position(instance):
    """JointState positions are radians/metres, never raw SDK scalars or speeds."""
    port=instance.get('ports',{}).get('position',{})
    return (instance.get('kind')=='actuator' and port.get('semantic')=='joint_state'
            and port.get('unit') in {'rad','m'})


def main(args=None, *, node_name='morla_runtime', required_plugin=None,
         publish_joint_states=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    from std_msgs.msg import String
    from morla_interfaces.msg import CommandEnvelope, CommandReceipt, ComponentHealth
    from morla_interfaces.srv import RuntimeOperation, ControlAuthority

    rclpy.init(args=args)

    class RuntimeNode(Node):
        def __init__(self):
            super().__init__(node_name)
            self.declare_parameter('resolved_path', '')
            self.declare_parameter('connect_inputs', False)
            self.declare_parameter('diagnostic_log_root', '')
            self.host=None
            path = self.get_parameter('resolved_path').value
            resolved = json.loads(Path(path).read_text())
            self.runtime = RuntimeSession(resolved,diagnostic_root=self.get_parameter('diagnostic_log_root').value or None)
            self.joint_publishers = {}
            self.last_samples = {}
            if publish_joint_states:
                from sensor_msgs.msg import JointState
                self.joint_publishers = {
                    key: self.create_publisher(JointState, '~/' + key + '/joint_states', 10)
                    for key, instance in self.runtime.instances.items()
                    if has_si_joint_position(instance)
                }
            self.twist_publishers = {}
            self.odom_publishers = {}
            if publish_joint_states:
                from geometry_msgs.msg import TwistStamped
                from nav_msgs.msg import Odometry
                for key, instance in self.runtime.instances.items():
                    ports=instance.get('ports',{})
                    if ports.get('velocity',{}).get('unit')=='m_rad/s' and ports['velocity'].get('dim')==2:
                        self.twist_publishers[key]=self.create_publisher(TwistStamped,'~/'+key+'/velocity',10)
                        if ports.get('odometry',{}).get('dim')==7:
                            self.odom_publishers[key]=self.create_publisher(Odometry,'~/'+key+'/odometry',10)
            self.image_publishers = {}
            if publish_joint_states:
                from sensor_msgs.msg import CompressedImage
                for key, instance in self.runtime.instances.items():
                    for port, definition in instance.get('ports',{}).items():
                        if definition.get('direction')=='output' and definition.get('semantic')=='image':
                            self.image_publishers[(key,port)]=self.create_publisher(CompressedImage,'~/'+key+'/'+port+'/compressed',1)
            self.state_pub = self.create_publisher(String, '~/state', 10)
            self.receipts = self.create_publisher(CommandReceipt, '~/receipts', 10)
            self.health_pub = self.create_publisher(ComponentHealth, '~/health', 10)
            self.create_subscription(CommandEnvelope, '~/commands', self.command, 1)
            self.create_service(RuntimeOperation, '~/operation', self.operation)
            self.create_service(ControlAuthority, '~/authority', self.authority)
            if self.get_parameter('connect_inputs').value:
                from morla_input.adapters.ingress import InputIngressAdapter
                self.runtime.adapters.update({key:InputIngressAdapter(instance,self.runtime.clock)
                    for key,instance in self.runtime.instances.items() if instance['kind']=='input'})
            try:
                self.runtime.start()
                if self.get_parameter('connect_inputs').value:
                    from morla_teleop.session import ControlHost
                    self.host=ControlHost(self.runtime)
                    self.host.connect()
            except Exception:
                if self.host is not None:self.host.close()
                self.runtime.stop()
                raise
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
                result = self.runtime.invoke(req.operation, req.request_id, list(req.component_ids) or None)
                reply.status, reply.error = result['status'], result.get('error', '')
            except Exception as exc:
                reply.status, reply.error = 'FAILED', str(exc)
            return reply

        def authority(self, req, reply):
            try:
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
            if self.host is not None and self.host.inputs is not None:
                state['input_processes']=self.host.inputs.supervisor.state()
            # Image bytes stay out of the JSON health state. Read only cached
            # adapter samples; capture/encoding belongs to the device worker.
            for key in {key for key,port in self.image_publishers}:
                sample=self.runtime.adapters[key].read(self.runtime.clock())
                for port, value in sample.get('ports',{}).items():
                    publisher=self.image_publishers.get((key,port))
                    stamp=value.get('sample_ns',sample.get('sample_ns',0))
                    if publisher is None or not value.get('valid') or not value.get('data') or value.get('encoding') not in ('jpeg','png'):
                        continue
                    if stamp<=self.last_samples.get((key,port),-1):continue
                    from sensor_msgs.msg import CompressedImage
                    image=CompressedImage();image.format=value['encoding'];image.data=bytes(value['data'])
                    ros_stamp=self.get_clock().now().nanoseconds-(self.runtime.clock()-stamp)
                    image.header.stamp.sec=ros_stamp//1_000_000_000
                    image.header.stamp.nanosec=ros_stamp%1_000_000_000
                    image.header.frame_id=key+'_optical'
                    publisher.publish(image);self.last_samples[(key,port)]=stamp
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
                if key in self.twist_publishers and value.get('ready'):
                    velocity=value.get('ports',{}).get('velocity',{})
                    stamp=velocity.get('sample_ns',value.get('sample_ns',0))
                    if velocity.get('valid') and stamp>self.last_samples.get(key,-1):
                        from geometry_msgs.msg import TwistStamped
                        message=TwistStamped()
                        ros_stamp=self.get_clock().now().nanoseconds-(self.runtime.clock()-stamp)
                        message.header.stamp.sec=ros_stamp//1_000_000_000
                        message.header.stamp.nanosec=ros_stamp%1_000_000_000
                        message.header.frame_id=key+'_base'
                        message.twist.linear.x,message.twist.angular.z=velocity['values']
                        self.twist_publishers[key].publish(message)
                        pose=value.get('ports',{}).get('odometry',{})
                        if key in self.odom_publishers and pose.get('valid'):
                            from nav_msgs.msg import Odometry
                            odometry=Odometry();odometry.header=message.header
                            odometry.header.frame_id=key+'_odom';odometry.child_frame_id=key+'_base'
                            q=pose['values'];position=odometry.pose.pose.position;orientation=odometry.pose.pose.orientation
                            position.x,position.y,position.z=q[:3]
                            orientation.x,orientation.y,orientation.z,orientation.w=q[3:]
                            odometry.twist.twist=message.twist
                            # SDK gives no covariance: explicitly mark unknown.
                            for axis in range(6):
                                odometry.pose.covariance[axis*7]=1e6
                                odometry.twist.covariance[axis*7]=1e6
                            self.odom_publishers[key].publish(odometry)
                        self.last_samples[key]=stamp
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
                    message.position = value.get('ports',{}).get('position',{}).get('values',value['values'])
                    self.joint_publishers[key].publish(message)
                    self.last_samples[key] = stamp

    node = None
    try:
        node = RuntimeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        # Humble can invalidate the wait set while SIGINT shuts the context.
        # Genuine ROS errors with a live context must still fail visibly.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            if node.host is not None:node.host.close()
            node.runtime.stop()
            node.destroy_node()
        rclpy.try_shutdown()


def joint_state_main(args=None):
    """Generic multi-component gateway with named JointState feedback outputs."""
    return main(args, publish_joint_states=True)
