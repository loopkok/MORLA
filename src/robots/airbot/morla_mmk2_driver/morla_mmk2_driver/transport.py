"""Bounded native MMK2 gRPC protocol. No high-level SDK startup side effects."""
import logging
import threading
import time
from .vendor import common_pb2 as c, mmk2_pb2 as m

log = logging.getLogger(__name__)


def stamp_ns(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000 or stamp.sec < 0:
        raise ValueError('MMK2_INVALID_DEVICE_TIMESTAMP')
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Transport:
    def __init__(self, parameters):
        import grpc
        from .vendor.mmk2_pb2_grpc import MMK2ServiceStub
        from .contract import COMMAND_RPC_TIMEOUT_DEFAULT_S
        self._rpc_error = grpc.RpcError
        self.p = parameters
        self.command_rpc_timeout_s = parameters.get('command_rpc_timeout_s', COMMAND_RPC_TIMEOUT_DEFAULT_S)
        self.channel = grpc.insecure_channel(f"{parameters['host']}:{parameters['port']}", options=[('grpc.max_receive_message_length', 32*1024*1024)])
        self.stub = MMK2ServiceStub(self.channel)
        self.stream = None
        self.closed = threading.Event()
        self.header = c.Header(caller_id=parameters['robot_name'])
        self.last_goal_rtt_s = None
        self._goal_rtt = {'count': 0, 'total_s': 0., 'max_s': 0., 'slow': 0, 'since': time.monotonic()}

    def _header(self):
        stamp=time.time_ns()
        return c.Header(stamp=c.Time(sec=stamp//10**9,nanosec=stamp%10**9))

    def open(self):
        self.registration = dict(self.stub.create_robot(c.ParamList(params={'robot_name': self.p['robot_name'], 'domain_id': str(self.p['domain_id'])}), timeout=self.p['rpc_timeout_s']).params)

    def states(self):
        # Long-lived stream is explicitly cancelled on close; receipt age is
        # checked separately by the command worker, even if the stream stalls.
        self.stream = self.stub.get_robot_state(self._header())
        if self.closed.is_set():
            self.stream.cancel()
        try:
            yield from self.stream
        except self._rpc_error as exc:
            if not self.closed.is_set():
                raise RuntimeError(f'MMK2_GET_ROBOT_STATE_RPC: {exc.code().name}: {exc.details()}') from exc

    def write(self, targets, rows):
        request = m.SetGoalRequest(header=self._header())
        from .contract import ROLES, role_of, mapping
        for key, q in targets.items():
            row = rows[key]; role = role_of(row); component = ROLES[role][0]
            goal = request.goals_map[component]
            if role == 'base':
                # Mapping velocity cannot use position offsets.
                signs, _ = mapping(row)
                goal.goal.twist3d.CopyFrom(c.Twist3D(x=q[0]*signs[0], omega=q[1]*signs[1]))
                goal.param.base_control.CopyFrom(m.BaseControlParams())
            else:
                signs, offsets = mapping(row)
                goal.goal.joint_state.CopyFrom(c.JointState(position=[(v-o)/s for v,s,o in zip(q,signs,offsets)]))
                if self.p['command_interface'] == 'trajectory':
                    scale = self.p['trajectory_scale']
                    goal.param.trajectory.CopyFrom(m.TrajectoryParams(max_velocity_scaling_factor=scale, max_acceleration_scaling_factor=scale, wait=False))
                else:
                    goal.param.forward_position.CopyFrom(m.ForwardPositionParams(max_velocity=row['parameters']['max_velocity']))
        if request.goals_map:
            # set_goal is a planning RPC (~0.7-1.2 s on the real controller),
            # not a read RPC; it needs its own deadline.
            timeout = self.command_rpc_timeout_s
            started = time.monotonic()
            try:
                reply = self.stub.set_goal(request, timeout=timeout)
            except self._rpc_error as exc:
                elapsed_ms = (time.monotonic() - started) * 1000
                components = ','.join(sorted(request.goals_map))
                raise RuntimeError(
                    f'MMK2_SET_GOAL_RPC: {exc.code().name}; components={components}; '
                    f'interface={self.p["command_interface"]}; elapsed_ms={elapsed_ms:.1f}; '
                    f'deadline_s={timeout:g}; {exc.details()}'
                ) from exc
            self._record_goal_rtt(time.monotonic() - started)
            if reply.value != c.GoalStatus.SUCCESS:
                # The controller rejects the whole request without saying which
                # goal was invalid. Dump the exact SDK targets so an
                # INVALID_TARGET can be attributed instead of guessed.
                detail = []
                for key, goal in request.goals_map.items():
                    if role_of(rows[key]) == 'base':
                        detail.append(f'{key}=twist({goal.goal.twist3d.x:+.4f},{goal.goal.twist3d.omega:+.4f})')
                    else:
                        detail.append(f'{key}={[round(v, 4) for v in goal.goal.joint_state.position]}')
                raise RuntimeError(f'MMK2_GOAL_REJECTED:{reply.value}:{reply.message}; '
                                   f'interface={self.p["command_interface"]}; sdk_targets={"; ".join(detail)}')

    def _record_goal_rtt(self, rtt_s):
        self.last_goal_rtt_s = rtt_s
        stats = self._goal_rtt
        stats['count'] += 1
        stats['total_s'] += rtt_s
        stats['max_s'] = max(stats['max_s'], rtt_s)
        if rtt_s > .5 * self.command_rpc_timeout_s:
            stats['slow'] += 1
        now = time.monotonic()
        if now - stats['since'] >= 30.:
            log.info('MMK2_SET_GOAL_RTT count=%d avg_ms=%.1f max_ms=%.1f slow=%d deadline_s=%g',
                     stats['count'], stats['total_s'] / stats['count'] * 1e3,
                     stats['max_s'] * 1e3, stats['slow'], self.command_rpc_timeout_s)
            self._goal_rtt = {'count': 0, 'total_s': 0., 'max_s': 0., 'slow': 0, 'since': now}

    def images(self, camera):
        return self.stub.get_image_once(m.GetImageRequest(header=self._header(), components=[camera,camera], image_types=['color','depth']), timeout=self.p['rpc_timeout_s'])

    def close(self):
        self.closed.set()
        if self.stream is not None:
            self.stream.cancel()
        self.channel.close()
