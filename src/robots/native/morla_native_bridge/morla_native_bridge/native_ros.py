"""Registered native ROS bridges. Discovery is pure; connecting is explicit.

Nero/Astral SDKs stay in their existing bounded child driver processes. All
command endpoints are remapped into a private runtime session namespace. ROS
names provide accidental-writer exclusion, not an untrusted-participant boundary.
"""
import copy
import math
import os
import threading
import time
from morla_runtime.process_executor import ProcessExecutor

_XHAND_NAMES = ('thumb_bend_joint', 'thumb_rota_joint1', 'thumb_rota_joint2', 'index_bend_joint',
                'index_joint1', 'index_joint2', 'mid_joint1', 'mid_joint2',
                'ring_joint1', 'ring_joint2', 'pinky_joint1', 'pinky_joint2')
_ASTRAL_ARM = ('shoulder_pitch', 'shoulder_roll', 'elbow_roll', 'elbow_pitch',
               'forearm_roll', 'wrist_pitch', 'wrist_roll')
_SUPPORTED = {'morla.nero.arm', 'morla.astral.arm', 'morla.astral.gripper',
              'morla.xhand.hand', 'morla.wuji.hand'}


def _node(package, executable, name, namespace, parameters, remaps):
    argv = ['ros2', 'run', package, executable, '--ros-args', '-r', '__node:=' + name,
            '-r', '__ns:=' + namespace]
    for key, value in sorted(parameters.items()):
        # ROS parameters are a single argv value, never interpolated into a shell.
        import json
        argv.extend(['-p', key + ':=' + json.dumps(value, allow_nan=False)])
    for source, target in sorted(remaps.items()):
        argv.extend(['-r', source + ':=' + target])
    return argv


def native_plan(device, instances, runtime_id):
    """Pure trusted argv and mapping generation; no sockets/SDK/ROS imports."""
    selected = [row for row in instances if row['instance_id'] in device['instances']]
    if not selected or any(row['plugin_id'] not in _SUPPORTED for row in selected):
        raise ValueError('NATIVE_BACKEND_UNSUPPORTED')
    import hashlib
    root = '/morla_sessions/s_' + runtime_id + '/d_' + hashlib.sha256(device['session_id'].encode()).hexdigest()[:16]
    settings = device.get('parameters', {})
    plan = {'argv': [], 'ports': {}, 'root': root, 'board_scoped': False}
    family = device.get('session_family', '')
    astral = family == 'astral_board' or all(row['plugin_id'].startswith('morla.astral.') for row in selected)
    if astral:
        # One process/SDK connection for all selected ports of the board.
        remaps = {}
        for role in ('left_arm', 'right_arm', 'astral', 'head', 'left_gripper', 'right_gripper'):
            for suffix in ('joint_commands', 'joint_states', 'command'):
                remaps['/' + role + '/' + suffix] = root + '/' + role + '/' + suffix
        params = {key: settings[key] for key in ('control_board_ip', 'board_cmd_port', 'local_ip', 'local_port') if key in settings}
        params.update(auto_ready=False, enable_full_body_cmd=False, enable_head_cmd=False,
                      enable_gripper_cmd=any(row['plugin_id'].endswith('gripper') for row in selected),
                      command_timeout_s=1.5,
                      managed_arm_sides=[row.get('parameters', {}).get('side', row.get('role', '')) for row in selected if row['plugin_id'].endswith('arm')] or ['none'],
                      managed_gripper_sides=[row.get('parameters', {}).get('side', row.get('role', '')) for row in selected if row['plugin_id'].endswith('gripper')] or ['none'])
        plan['argv'].append(_node('astral_robot_control', 'astral_robot_driver', 'driver', root, params, remaps))
        plan['board_scoped'] = True
    for row in selected:
        key, plugin = row['instance_id'], row['plugin_id']
        side = row.get('parameters', {}).get('side') or row.get('role')
        if side not in ('left', 'right'):
            raise ValueError('NATIVE_SIDE_REQUIRED: ' + key)
        local = root + '/' + key
        command_topic, state_topic = local + '/command', local + '/state'
        enable, stop = local + '/driver/enable', local + '/driver/estop'
        if plugin == 'morla.nero.arm':
            names = [side + '_joint' + str(i) for i in range(1, 8)]
            params = {key: settings[key] for key in ('can_channel', 'can_interface', 'firmware_version', 'native_j7_first', 'initial_position', 'workpos_waypoints_deg') if key in settings}
            # Do not invent a cold-start pose; the existing driver validates it.
            if 'initial_position' not in params:
                raise ValueError('NERO_INITIAL_POSITION_REQUIRED')
            params.update(arm_side=side, sdk_backend='hardware', stop_on_shutdown=True, command_timeout_s=0.5)
            remaps = {'/' + side + '_arm/joint_commands': command_topic,
                      '/' + side + '_arm/joint_states': state_topic,
                      '/teleop/armed': local + '/armed', '/teleop/disarm': local + '/disarm'}
            plan['argv'].append(_node('nero_robot_control', 'nero_robot_driver', 'driver', local, params, remaps))
        elif plugin.startswith('morla.astral.'):
            role = side + ('_gripper' if plugin.endswith('gripper') else '_arm')
            names = [side + '_gripper'] if plugin.endswith('gripper') else [side + '_' + name for name in _ASTRAL_ARM]
            command_topic = root + '/' + role + ('/command' if plugin.endswith('gripper') else '/joint_commands')
            state_topic = root + '/' + role + '/joint_states'
            enable, stop = root + '/driver/enable', root + '/driver/estop'
        elif plugin == 'morla.wuji.hand':
            names = [side + '_finger' + str(i) + '_joint' + str(j) for i in range(1, 6) for j in range(1, 5)]
            params = {key: settings[key] for key in ('serial_number', 'publish_rate', 'filter_cutoff_freq') if key in settings}
            params.update(hand_side=side, auto_enable=False, name_by_handedness=False)
            plan['argv'].append(_node('wujihand_driver', 'wujihand_driver_node', 'driver', local, params,
                                      {'joint_commands': command_topic, 'joint_states': state_topic}))
            enable, stop = local + '/set_enabled', local + '/set_enabled'
        else:
            names = list(_XHAND_NAMES)
            joints = row.get('joints', [])
            params = {'side': side, 'hand_id': int(settings.get('hand_id', 0)), 'state_topic': state_topic,
                      'command_topic': command_topic, 'status_topic': local + '/adapter/status', 'native_state_topic': local + '/native/state',
                      'native_command_topic': local + '/native/command', 'require_armed': True,
                      'joint_lower_limits': [joint['lower'] for joint in joints],
                      'joint_upper_limits': [joint['upper'] for joint in joints], 'command_timeout_s': 0.5}
            plan['argv'].append(_node('xhand_control_ros2', 'xhand_control_ros2_node', 'native_driver', local,
                                      {'port_name': settings.get('port_name', '/dev/ttyUSB0')},
                                      {'xhand_state': local + '/native/state', 'xhand_command': local + '/native/command'}))
            plan['argv'].append(_node('xhand_adapter', 'xhand_adapter', 'adapter', local, params,
                                      {'/teleop/armed': local + '/armed', '/teleop/disarm': local + '/disarm'}))
            enable, stop = local + '/armed', local + '/disarm'
        plan['ports'][key] = {'native_names': names, 'command': command_topic, 'state': state_topic,
                             'enable': enable, 'stop': stop, 'plugin_id': plugin, 'root': local,
                             'gripper_open_rad': float(settings.get(side + '_gripper_open_rad', 0.8)),
                             'gripper_closed_rad': float(settings.get(side + '_gripper_closed_rad', 0.0))}
    return plan


class NativeRosTransport:
    """Dedicated ROS executor thread and measured latest-value state cache."""
    def __init__(self, plan, lane_fd, resource_root, clock):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64, Bool, String
        from std_srvs.srv import Trigger
        self.rclpy, self.JointState, self.Float64, self.Bool, self.Trigger = rclpy, JointState, Float64, Bool, Trigger
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('native_bridge_' + str(os.getpid()) + '_' + str(id(self)))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.plan, self.clock = plan, clock
        self.samples, self.publishers, self.clients, self.flags = {}, {}, {}, {}
        self.statuses = {}
        self.source_clocks = {}
        self.source_faults = set()
        self.processes = ProcessExecutor()
        self.lock = threading.Lock()
        for key, port in plan['ports'].items():
            if port['plugin_id'] in ('morla.nero.arm', 'morla.xhand.hand'):
                status_path = '/driver/status' if port['plugin_id'] == 'morla.nero.arm' else '/adapter/status'
                self.node.create_subscription(String, port['root'] + status_path, lambda msg, key=key: self._status(key, msg), 1)
            self.node.create_subscription(JointState, port['state'], lambda msg, key=key: self._sample(key, msg), qos_profile_sensor_data)
            message_type = Float64 if port['plugin_id'].endswith('gripper') else JointState
            self.publishers[key] = self.node.create_publisher(message_type, port['command'], qos_profile_sensor_data)
            if port['plugin_id'] == 'morla.xhand.hand':
                self.flags[(key, 'enable')] = self.node.create_publisher(Bool, port['enable'], 1)
                self.flags[(key, 'stop')] = self.node.create_publisher(Bool, port['stop'], 1)
            elif port['plugin_id'] == 'morla.wuji.hand':
                from wujihand_msgs.srv import SetEnabled
                self.clients[(key, 'enable')] = self.node.create_client(SetEnabled, port['enable'])
                self.clients[(key, 'stop')] = self.clients[(key, 'enable')]
            else:
                for operation in ('enable', 'stop'):
                    self.clients[(key, operation)] = self.node.create_client(Trigger, port[operation])
                for operation, suffix in (('home', 'home'), ('workpos', 'position')):
                    self.clients[(key, operation)] = self.node.create_client(Trigger, port['enable'].rsplit('/', 1)[0] + '/' + suffix)
        self.thread.start()
        env = os.environ.copy()
        env['MORLA_RUNTIME_LOCK_FD'], env['MORLA_RESOURCE_ROOT'] = str(lane_fd), str(resource_root)
        try:
            for argv in plan['argv']:
                self.processes.start(argv, trusted=True, env=env, pass_fds=(lane_fd,))
        except Exception:
            self.close()
            raise

    def _status(self, key, message):
        import json
        try:
            payload = json.loads(message.data)
            with self.lock:
                self.statuses[key] = (self.clock(), payload)
        except (ValueError, TypeError):
            pass

    def _sample(self, key, message):
        stamp = message.header.stamp
        if not isinstance(stamp.sec, int) or not isinstance(stamp.nanosec, int) or not 0 <= stamp.nanosec < 1_000_000_000:
            return
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if source_ns <= 0:
            return
        with self.lock:
            previous = self.source_clocks.get(key)
            if previous is not None and source_ns < previous:
                self.source_faults.add(key)
                self.samples[key] = {'ready': False, 'sample_ns': self.clock(), 'source': 'hardware',
                                     'measured': False, 'reason': 'SOURCE_CLOCK_REGRESSION'}
                return
            if key in self.source_faults or previous is not None and source_ns == previous:
                return
        names, values = list(message.name), list(message.position)
        expected = self.plan['ports'][key]['native_names']
        if len(names) != len(set(names)) or len(values) != len(names) or set(names) != set(expected):
            return
        mapping = dict(zip(names, values))
        values = [mapping[name] for name in expected]
        port = self.plan['ports'][key]
        gripper_echo = port['plugin_id'] == 'morla.astral.gripper'
        if gripper_echo:
            opened, closed = port['gripper_open_rad'], port['gripper_closed_rad']
            if opened == closed:
                return
            values = [(value - opened) / (closed - opened) for value in values]
        if not all(math.isfinite(value) for value in values):
            return
        with self.lock:
            self.source_clocks[key] = source_ns
            self.samples[key] = {'ready': not gripper_echo, 'sample_ns': self.clock(),
                                 'source': 'command_echo' if gripper_echo else 'hardware',
                                 'measured': not gripper_echo, 'synthetic': False, 'values': values,
                                 'source_ns': source_ns,
                                 'reason': 'GRIPPER_HAS_NO_MEASURED_FEEDBACK' if gripper_echo else '',
                                 'ports': {'position': {'values': values, 'unit': 'ratio' if gripper_echo else 'rad', 'valid': not gripper_echo,
                                                       'sample_ns': self.clock(), 'source': 'command_echo' if gripper_echo else 'hardware', 'measured': not gripper_echo}}}
            velocities = list(message.velocity)
            if len(velocities) == len(names) and all(math.isfinite(value) for value in velocities):
                velocity_map = dict(zip(names, velocities))
                self.samples[key]['ports']['velocity'] = {'values': [velocity_map[name] for name in expected],
                    'unit': 'rad/s', 'valid': True, 'sample_ns': self.clock(), 'source': 'hardware', 'measured': True}

    def health(self, key):
        with self.lock:
            return copy.deepcopy(self.samples.get(key, {'ready': False, 'sample_ns': 0, 'source': 'hardware',
                                                       'measured': False, 'synthetic': False, 'values': []}))

    def operation(self, key, operation, timeout=None):
        timeout = (5.0 if operation == "enable" else 0.5) if timeout is None else timeout
        request_started_ns = self.clock()
        if (key, operation) in self.flags:
            # A Bool publish has no execution ACK; rely on measured state for
            # readiness, and never claim verified physical stop from this alone.
            message = self.Bool()
            message.data = True
            self.flags[(key, operation)].publish(message)
            began = self.clock()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                with self.lock:
                    stamp, status = self.statuses.get(key, (0, {}))
                if stamp >= began and status.get('armed') is (operation == 'enable'):
                    return True
                time.sleep(.002)
            raise TimeoutError('XHAND_GATE_ACK_TIMEOUT')
        client = self.clients[(key, operation)]
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError('NATIVE_SERVICE_UNAVAILABLE')
        request = client.srv_type.Request()
        if self.plan['ports'][key]['plugin_id'] == 'morla.wuji.hand':
            request.finger_id, request.joint_id, request.enabled = 255, 255, operation == 'enable'
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.002)
        if not future.done():
            future.cancel()
            raise TimeoutError('NATIVE_OPERATION_TIMEOUT')
        response = future.result()
        if not response.success:
            return False
        return True

    def submit(self, key, command):
        port = self.plan['ports'][key]
        if port['plugin_id'].endswith('gripper'):
            message = self.Float64()
            message.data = command['values'][0]
        else:
            message = self.JointState()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.name, message.position = port['native_names'], list(command['values'])
        self.publishers[key].publish(message)
        return {'accepted': True, 'status': 'dispatched_transport', 'execution_ack': False}

    def close(self):
        self.processes.close()
        self.executor.shutdown(timeout_sec=0.5)
        self.thread.join(timeout=0.5)
        self.node.destroy_node()


class NativeDeviceSession:
    def __init__(self, device, instances, runtime, transport_factory=NativeRosTransport):
        self.device, self.instances, self.runtime = device, instances, runtime
        self.transport_factory, self.transport = transport_factory, None
        self.active_ports = set()
        self.enabled = False
        self.closed = False

    def configure(self):
        if self.transport is None:
            plan = native_plan(self.device, self.instances, self.runtime.session_id)
            self.transport = self.transport_factory(plan, self.runtime.resources.lane_fd,
                                                    self.runtime.resources.root, self.runtime.clock)

    def operation(self, key, operation):
        if self.device.get('session_family') == 'astral_board':
            if operation == 'enable' and self.enabled:
                return True
        if operation in ('enable', 'home', 'workpos'):
            # A timed-out enable can still take effect later in a vendor SDK.
            # The compensating stop must run even without a successful reply.
            self.enabled = True
        result = self.transport.operation(key, operation)
        if result and operation == 'enable':
            self.active_ports.add(key)
            self.enabled = True
        if result and operation == 'stop':
            self.active_ports.clear()
            self.enabled = False
        return result

    def close(self):
        if not self.closed and self.transport is not None:
            self.transport.close()
            self.closed = True


class NativeRosAdapter:
    backend = 'real'
    device_scoped_stop = True
    def __init__(self, instance, session):
        self.instance, self.session = instance, session
        self.key = instance['instance_id']
        self.active = False

    def configure(self, context):
        self.session.configure()

    def health(self, now_ns):
        sample=dict(self.session.transport.health(self.key))
        processes=getattr(self.session.transport,'processes',None)
        if processes is not None:sample['processes']=processes.state()
        return sample

    def read(self, now_ns):
        return self.health(now_ns)

    def prepare(self):
        # Explicit cold-start enable for Nero; never hidden inside configure.
        return self.session.operation(self.key, 'enable')

    def activate(self):
        result = self.session.operation(self.key, 'enable')
        self.active = bool(result)
        return result

    def submit(self, command):
        if not self.active:
            raise RuntimeError('ADAPTER_DISARMED')
        return self.session.transport.submit(self.key, command)

    def operate(self, operation):
        if operation not in self.instance.get('operations', {}):
            raise RuntimeError('UNSUPPORTED_OPERATION')
        if operation == 'disable':
            return self.stop()
        result = self.session.operation(self.key, operation)
        if result and operation in ('home', 'workpos'):
            return {'accepted': True, 'completed': False, 'phase': 'native_request_dispatched',
                    'requires_operator_confirmation': True,
                    'reason': 'Native staged trajectory requires measured stage arrival and explicit continuation'}
        return result

    def stop(self):
        if self.session.transport is None or not self.session.enabled:
            self.active = False
            return True
        result = self.session.operation(self.key, 'stop')
        self.active = False
        return result

    def close(self):
        self.session.close()


def make_native_adapters(runtime, transport_factory=NativeRosTransport):
    result = {}
    rows = list(runtime.instances.values())
    for device in runtime.resolved.get('device_sessions', []):
        if device.get('backend') != 'real':
            continue
        session = NativeDeviceSession(device, rows, runtime, transport_factory)
        for key in device['instances']:
            if runtime.instances[key]['plugin_id'] not in _SUPPORTED:
                continue
            result[key] = NativeRosAdapter(runtime.instances[key], session)
    return result
