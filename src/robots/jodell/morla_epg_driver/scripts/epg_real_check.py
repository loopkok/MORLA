#!/usr/bin/env python3
"""Guarded bench check for one real EPG50-060 behind the MORLA EPG driver.

Two phases, both through the installed ROS services/topics (no direct SDK use):

1. read-only (default): verify the node runs the given snapshot, print status
   and the device-reported width, and run `prepare`. Nothing is enabled.
2. motion (only with --confirm-motion and --target-width): activate, acquire
   authority, stream commands at 10 Hz above the 0.3 s device command watchdog,
   observe feedback, then `stop`.

Motion refuses unless the deployment declares verified/stop_verified and the
parameters set activation_motion_verified/position_feedback_verified/
device_watchdog_verified — the same gates the adapter enforces. It also refuses
a target more than 5 mm from the reported width.

Example:
  python3 src/robots/jodell/morla_epg_driver/scripts/epg_real_check.py \
    --resolved /tmp/epg.real.json
  python3 .../epg_real_check.py --resolved /tmp/epg.real.json \
    --confirm-motion --target-width 0.045
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

STEP_LIMIT_M = .005


def load_snapshot(path):
    snapshot = json.loads(Path(path).resolve(strict=True).read_text())
    from morla_core.composition import verify_resolved
    verify_resolved(snapshot)
    instances = snapshot.get('instances', [])
    actuators = [item for item in instances if item.get('kind') == 'actuator']
    if len(instances) != 1 or len(actuators) != 1:
        raise SystemExit('Expected an exact single-actuator snapshot')
    instance = actuators[0]
    if instance.get('backend') != 'real':
        raise SystemExit(
            'This tool only drives a real EPG snapshot; use the fake gateway smoke for fake')
    if 'epg' not in instance.get('plugin_id', ''):
        raise SystemExit(
            'Snapshot component is not the EPG plugin: ' + str(instance.get('plugin_id')))
    return snapshot, instance


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--resolved', required=True, help='compiled single-EPG real snapshot')
    parser.add_argument('--node', default='/epg_driver')
    parser.add_argument('--target-width', type=float,
                        help='absolute opening width in metres (0 closed, 0.05 open)')
    parser.add_argument('--hold-s', type=float, default=2.0,
                        help='seconds to stream the target (default 2)')
    parser.add_argument('--stream-hz', type=float, default=10.0)
    parser.add_argument('--confirm-motion', action='store_true',
                        help='required to enable/activate; the enable stroke may move the fingers')
    args = parser.parse_args(argv)
    snapshot, instance = load_snapshot(args.resolved)
    component = instance['instance_id']
    params = instance.get('parameters') or {}
    limits = instance['joints'][0]
    lower, upper = (limits.get('limits') or [limits.get('lower'), limits.get('upper')])[:2]
    flags = {
        'deployment_verified': bool(instance.get('deployment_verified')),
        'watchdog_verified': bool(instance.get('watchdog_verified')),
        'activation_motion_verified': params.get('activation_motion_verified') is True,
        'position_feedback_verified': params.get('position_feedback_verified') is True,
        'device_watchdog_verified': params.get('device_watchdog_verified') is True,
    }
    print('snapshot: component=%s model=%s port=%s slave=%s baud=%s'
          % (component, params.get('model'), params.get('port'),
             params.get('slave_id'), params.get('baudrate')), flush=True)
    print('verification flags: ' + json.dumps(flags), flush=True)
    if args.confirm_motion and not all(flags.values()):
        raise SystemExit(
            'Refusing motion: declare verified/stop_verified and the four parameter flags '
            'only after the real hardware checks they assert')
    if args.confirm_motion and args.target_width is None:
        raise SystemExit('--target-width is required with --confirm-motion')
    if args.target_width is not None and not lower <= args.target_width <= upper:
        raise SystemExit('Target %.4f m outside [%s, %s]' % (args.target_width, lower, upper))

    import rclpy
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from morla_interfaces.msg import CommandEnvelope, CommandReceipt, ComponentTarget
    from morla_interfaces.srv import RuntimeOperation, ControlAuthority

    rclpy.init()
    node = rclpy.create_node('morla_epg_real_check_' + str(int(time.time())))
    prefix = args.node.rstrip('/')
    state = {}
    receipts = []
    permit = None
    activated = False
    stopped = False
    request_prefix = 'epg-real-' + str(int(time.time()))
    owner = 'epg_real_bench'

    def update(message):
        state.clear()
        state.update(json.loads(message.data))

    node.create_subscription(String, prefix + '/state', update, 10)
    node.create_subscription(CommandReceipt, prefix + '/receipts', receipts.append, 10)
    publisher = node.create_publisher(CommandEnvelope, prefix + '/commands', 10)
    operations = node.create_client(RuntimeOperation, prefix + '/operation')
    authority = node.create_client(ControlAuthority, prefix + '/authority')
    parameters = node.create_client(GetParameters, prefix + '/get_parameters')

    def wait(predicate, label, timeout=5.):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(label + ': ' + json.dumps(state, ensure_ascii=False)[:1500])
            rclpy.spin_once(node, timeout_sec=.02)

    def call(client, request):
        if not client.wait_for_service(timeout_sec=5.):
            raise TimeoutError('Service unavailable: ' + client.srv_name)
        future = client.call_async(request)
        wait(future.done, client.srv_name)
        return future.result()

    def operation(name):
        request = RuntimeOperation.Request()
        request.session_id = state['session_id']
        request.request_id = request_prefix + '-' + name
        request.operation = name
        reply = call(operations, request)
        if reply.status == 'FAILED':
            raise RuntimeError(name + ': ' + reply.error)
        return reply

    def auth(name, ttl_ns=5_000_000_000):
        request = ControlAuthority.Request()
        request.session_id = state['session_id']
        request.request_id = request_prefix + '-auth-' + name
        request.operation = name
        request.producer = owner
        request.resources = [component]
        request.ttl_ns = ttl_ns
        request.token = permit.token if permit else ''
        reply = call(authority, request)
        if not reply.accepted:
            raise RuntimeError(name + ': ' + reply.reason)
        return reply

    def width():
        value = state.get('components', {}).get(component, {})
        values = value.get('values') or []
        return values[0] if values else None

    try:
        request = GetParameters.Request()
        request.names = ['resolved_path']
        actual = call(parameters, request).values[0].string_value
        if Path(actual).resolve() != Path(args.resolved).resolve(strict=True):
            raise SystemExit('Running node uses a different resolved_path: ' + actual)
        wait(lambda: state.get('components', {}).get(component, {}).get('ready') is True,
             'healthy EPG state')
        print(json.dumps({'phase': 'read-only', 'status': state.get('status'),
                          'health': state['components'][component]},
                         ensure_ascii=False), flush=True)
        operation('prepare')
        print(json.dumps({'phase': 'read-only', 'prepare': 'RUNNING',
                          'status': state.get('status')}, ensure_ascii=False), flush=True)
        if not args.confirm_motion:
            print('PASS read-only: no enable, no motion, no register write')
            return 0

        current = width()
        if current is None:
            raise SystemExit('No device-reported width available; cannot bound the step')
        step = args.target_width - current
        print(json.dumps({'current_width_m': current, 'target_width_m': args.target_width,
                          'step_m': step}, ensure_ascii=False), flush=True)
        if abs(step) > STEP_LIMIT_M:
            raise SystemExit('Requested step %.4f m exceeds %.4f m; move in smaller increments'
                             % (step, STEP_LIMIT_M))
        if abs(step) < 1e-4:
            raise SystemExit('Target is within measurement resolution; pick a different width')

        print('WARNING: activate may run an enable/search stroke. Keep the gripper unloaded, '
              'within its 50 mm stroke, and keep the 24 V switch or physical stop in reach.',
              flush=True)
        operation('activate')
        wait(lambda: state.get('status') == 'ACTIVE', 'activate', timeout=30.)
        activated = True
        permit = auth('acquire')
        wait(lambda: publisher.get_subscription_count() > 0, 'command DDS match')
        names = [joint['name'] for joint in instance['joints']]
        units = [joint.get('unit', 'm') for joint in instance['joints']]
        accepted = 0
        rejected = []
        deadline = time.monotonic() + args.hold_s
        sequence = 1
        period = 1. / args.stream_hz
        while time.monotonic() < deadline:
            message = CommandEnvelope()
            message.session_id = state['session_id']
            message.producer = owner
            message.token = permit.token
            message.epoch = permit.epoch
            message.sequence = sequence
            message.issued_ns = time.monotonic_ns()
            message.ttl_ns = 500_000_000
            target = ComponentTarget()
            target.component_id = component
            target.mode = 'position'
            target.names = names
            target.units = units
            target.values = [args.target_width]
            message.commands = [target]
            before = len(receipts)
            publisher.publish(message)
            sequence += 1
            end = time.monotonic() + min(period, max(.01, deadline - time.monotonic()))
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.01)
            if len(receipts) > before:
                receipt = receipts[before]
                if receipt.accepted and list(receipt.dispatched) == [component]:
                    accepted += 1
                else:
                    rejected.append(receipt.reason)
        print(json.dumps({'phase': 'motion', 'accepted_commands': accepted,
                          'rejected': rejected, 'width_m': width()},
                         ensure_ascii=False), flush=True)
        if not accepted:
            raise RuntimeError('No command was accepted')
        if rejected:
            raise RuntimeError('Commands rejected: ' + json.dumps(rejected))
        wait(lambda: width() is not None
             and math.isclose(width(), args.target_width, abs_tol=5e-4),
             'target feedback', timeout=5.)
        operation('stop')
        stopped = True
        print(json.dumps({'phase': 'motion', 'status': state.get('status'),
                          'final_width_m': width()}, ensure_ascii=False), flush=True)
        print('PASS motion: target accepted, feedback reached, stop confirmed')
        return 0
    finally:
        if activated and not stopped:
            try:
                operation('stop')
                stopped = True
            except Exception as exc:
                print('CLEANUP_UNCONFIRMED stop: ' + str(exc), flush=True)
        if permit is not None and not stopped:
            try:
                auth('revoke')
            except Exception as exc:
                print('CLEANUP_UNCONFIRMED revoke: ' + str(exc), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
