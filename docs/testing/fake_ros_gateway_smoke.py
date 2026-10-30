#!/usr/bin/env python3
"""Exercise installed ROS command/service transport against one fake actuator.

Run only in an isolated ROS domain with a freshly launched fake single-device
driver. No real backend is supported. This host-side check has not been run over
DDS on the documentation authoring machine; --help needs no ROS installation.

2026-10-30: retained as a single-actuator DDS transport probe. It does not test
ControlHost input bindings, process factories, filters or multi-arm composition;
run U01-U08 in host_build_and_hardware_acceptance.md for those paths. Generic
system.launch.py uses /morla_runtime; independent drivers keep their node names.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time
import uuid


def fake_snapshot(path):
    data = json.loads(Path(path).read_text())
    instances = data.get('instances', [])
    if (len(instances) != 1 or instances[0].get('kind') != 'actuator'
            or instances[0].get('backend') != 'fake'):
        raise ValueError('Only an exact single-actuator fake snapshot is accepted')
    return data


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resolved', required=True)
    parser.add_argument('--node', required=True, help='e.g. /morla_runtime (generic system) or /cr5_driver (single device)')
    parser.add_argument('--component', required=True, help='e.g. arm, hand or gripper')
    args = parser.parse_args(argv)
    path = Path(args.resolved).resolve(strict=True)
    snapshot = fake_snapshot(path)
    instance = snapshot['instances'][0]
    if instance['instance_id'] != args.component:
        raise ValueError('Component does not match snapshot')

    import rclpy
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from morla_interfaces.msg import CommandEnvelope, CommandReceipt, ComponentTarget
    from morla_interfaces.srv import RuntimeOperation, ControlAuthority
    from morla_core.composition import verify_resolved

    verify_resolved(snapshot)
    rclpy.init()
    node = rclpy.create_node('morla_host_fake_check_' + uuid.uuid4().hex[:8])
    prefix = args.node.rstrip('/')
    state = {}
    receipts = []
    request_prefix = 'host-fake-' + uuid.uuid4().hex
    owner = 'host_fake_test'
    permit = None

    def update(msg):
        state.clear()
        state.update(json.loads(msg.data))

    node.create_subscription(String, prefix + '/state', update, 10)
    node.create_subscription(CommandReceipt, prefix + '/receipts', receipts.append, 10)
    publisher = node.create_publisher(CommandEnvelope, prefix + '/commands', 10)

    def wait(predicate, label, timeout=5.):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(label + ': ' + json.dumps(state, ensure_ascii=False))
            rclpy.spin_once(node, timeout_sec=.02)

    def call(client, request):
        if not client.wait_for_service(timeout_sec=5.):
            raise TimeoutError('Service unavailable: ' + client.srv_name)
        future = client.call_async(request)
        wait(future.done, client.srv_name)
        return future.result()

    operations = node.create_client(RuntimeOperation, prefix + '/operation')
    authority = node.create_client(ControlAuthority, prefix + '/authority')
    parameters = node.create_client(GetParameters, prefix + '/get_parameters')

    def operation(name):
        request = RuntimeOperation.Request()
        request.session_id = state['session_id']
        request.request_id = request_prefix + '-' + name
        request.operation = name
        reply = call(operations, request)
        if reply.status == 'FAILED':
            raise RuntimeError(name + ': ' + reply.error)
        return reply

    def auth(name):
        request = ControlAuthority.Request()
        request.session_id = state['session_id']
        request.request_id = request_prefix + '-' + name
        request.operation = name
        request.producer = owner
        request.resources = [args.component]
        request.ttl_ns = 5_000_000_000
        request.token = permit.token if permit else ''
        reply = call(authority, request)
        if not reply.accepted:
            raise RuntimeError(name + ': ' + reply.reason)
        return reply

    try:
        request = GetParameters.Request()
        request.names = ['resolved_path']
        actual = call(parameters, request).values[0].string_value
        if Path(actual).resolve() != path:
            raise ValueError('Running node uses a different resolved_path')
        wait(lambda: args.component in state.get('components', {}), 'initial fake state')
        if (state['status'] != 'READY_DISARMED'
                or any(not h.get('synthetic') or h.get('measured')
                       for h in state['components'].values())):
            raise ValueError('Fresh disarmed synthetic-only node required; real state is refused')
        operation('prepare')
        wait(lambda: state.get('status') == 'READY_DISARMED', 'prepare')
        operation('activate')
        wait(lambda: state.get('status') == 'ACTIVE', 'activate')
        permit = auth('acquire')
        wait(lambda: publisher.get_subscription_count() > 0, 'command DDS match')
        values = list(state['components'][args.component]['values'])
        first = instance['joints'][0]
        limits = first.get('limits', {})
        upper = first.get('upper', limits.get('upper') if isinstance(limits, dict) else limits[1])
        lower = first.get('lower', limits.get('lower') if isinstance(limits, dict) else limits[0])
        candidate = values[0] + .001
        if upper is not None and candidate > upper:
            candidate = values[0] - .001
        if lower is not None and candidate < lower:
            raise ValueError('No small fake test increment within first joint limits')
        values[0] = candidate

        def message(sequence):
            msg = CommandEnvelope()
            msg.session_id = state['session_id']
            msg.producer = owner
            msg.token = permit.token
            msg.epoch = permit.epoch
            msg.sequence = sequence
            msg.issued_ns = time.monotonic_ns()
            msg.ttl_ns = 500_000_000
            target = ComponentTarget()
            target.component_id = args.component
            target.mode = 'position'
            target.names = [j['name'] for j in instance['joints']]
            target.units = [j.get('unit', 'rad') for j in instance['joints']]
            target.values = values
            msg.commands = [target]
            return msg

        def send(msg, label, expected=None):
            count = len(receipts)
            publisher.publish(msg)
            wait(lambda: len(receipts) > count, label)
            result = receipts[count]
            if expected is None:
                assert result.accepted and list(result.dispatched) == [args.component], result.reason
            else:
                assert not result.accepted and not result.dispatched, label
                assert expected in result.reason, (label, result.reason, expected)
            print(json.dumps({'case': label, 'accepted': result.accepted,
                              'reason': result.reason, 'dispatched': list(result.dispatched)},
                             ensure_ascii=False), flush=True)

        good = message(1)
        send(good, 'valid fake target')
        wait(lambda: all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(
            state['components'][args.component]['values'], values)), 'fake target feedback')
        # Post-rollback submit checks only: estop latch, component subset/duplicates and
        # adapter acceptance. Token/epoch/sequence/TTL are carried but not enforced.
        cases = [
            ('duplicate component', 'COMMAND_RESOURCES', lambda m: m.commands.append(
                copy.deepcopy(m.commands[0]))),
            ('unknown component', 'COMMAND_RESOURCES', lambda m: setattr(
                m.commands[0], 'component_id', 'not-a-component')),
        ]
        for sequence, (label, expected, mutate) in enumerate(cases, 2):
            auth('renew')
            msg = message(sequence)
            mutate(msg)
            send(msg, label, expected)
        operation('estop')
        send(message(10), 'submit while estop latched', 'RESET_REQUIRED')
        operation('reset')
        wait(lambda: state.get('status') == 'READY_DISARMED', 'reset')
        send(message(11), 'submit after reset without activate', 'ADAPTER_DISARMED')
        permit = None
        print('PASS installed fake ROS services, command/receipt transport, feedback, '
              'and the retained rejection cases (resource subset, estop latch, disarmed adapter)')
    finally:
        if permit is not None:
            try:
                auth('revoke')
            except Exception as exc:
                print('CLEANUP_UNCONFIRMED: ' + str(exc), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
