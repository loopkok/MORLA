"""Bounded native MMK2 gRPC protocol. No high-level SDK startup side effects."""
import threading
import time
from .vendor import common_pb2 as c, mmk2_pb2 as m


def stamp_ns(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000 or stamp.sec < 0:
        raise ValueError('MMK2_INVALID_DEVICE_TIMESTAMP')
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Transport:
    def __init__(self, parameters):
        import grpc
        from .vendor.mmk2_pb2_grpc import MMK2ServiceStub
        self.p = parameters
        self.channel = grpc.insecure_channel(f"{parameters['host']}:{parameters['port']}", options=[('grpc.max_receive_message_length', 32*1024*1024)])
        self.stub = MMK2ServiceStub(self.channel)
        self.stream = None
        self.closed = threading.Event()
        self.header = c.Header(caller_id=parameters['robot_name'])

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
        yield from self.stream

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
            reply = self.stub.set_goal(request, timeout=self.p['rpc_timeout_s'])
            if reply.value != c.GoalStatus.SUCCESS:
                raise RuntimeError(f'MMK2_GOAL_REJECTED:{reply.value}:{reply.message}')

    def images(self, camera):
        return self.stub.get_image_once(m.GetImageRequest(header=self._header(), components=[camera,camera], image_types=['color','depth']), timeout=self.p['rpc_timeout_s'])

    def close(self):
        self.closed.set()
        if self.stream is not None:
            self.stream.cancel()
        self.channel.close()
