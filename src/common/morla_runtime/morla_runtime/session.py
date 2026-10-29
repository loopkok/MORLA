"""ROS-free composable runtime with fail-closed, monotonic command authority."""
import copy
import math
import secrets
import threading
import time
import uuid

from .adapters import FakeAdapter, Ros2ControlFakeAdapter
from .resource_manager import ResourceManager


class RuntimeErrorCode(RuntimeError):
    pass


class RuntimeSession:
    def __init__(self, resolved, resource_root=None, clock=None, adapters=None, registry=None, allow_unverified_test_fixture=False):
        if 'resolved_hash' not in resolved and not allow_unverified_test_fixture:
            raise RuntimeErrorCode('RESOLVED_HASH_REQUIRED')
        if 'resolved_hash' in resolved:
            from morla_core.composition.compiler import verify_resolved
            verify_resolved(resolved, registry=registry)
        self.resolved = copy.deepcopy(resolved)
        self.clock = clock or time.monotonic_ns
        self.session_id = uuid.uuid4().hex
        self.status = 'VALIDATED'
        self.estop_latched = False
        self.epoch = 0
        self.lease = None
        self.sequence = {}
        self.operations = {}
        self.events = []
        self.lock = threading.RLock()
        self.resources = ResourceManager(resource_root)
        self.instances = {x['instance_id']: x for x in resolved.get('instances', [])}
        self.adapters = dict(adapters or {})
        self.max_age_ns = int(resolved.get('safety_policy', {}).get('max_sample_age_ns', 500_000_000))
        self._last_now = self.clock()
        self._event_callbacks = []
        self._command_callbacks = []
        self._operation_generation = 0
        self._pending_operation = None

    def _now(self):
        now = self.clock()
        if now < self._last_now:
            self._revoke()
            self.status = 'FAULTED'
            raise RuntimeErrorCode('MONOTONIC_CLOCK_REGRESSION')
        self._last_now = now
        return now

    def _event(self, kind, **fields):
        event = {'event': kind, 'session_id': self.session_id, 'monotonic_ns': self.clock(), **fields}
        self.events.append(event)
        # Preserve a bounded in-memory ring; applications can persist event snapshots.
        if len(self.events) > 4096:
            del self.events[:1024]
        for callback in self._event_callbacks:
            try:
                callback(copy.deepcopy(event))
            except Exception:
                pass  # Observers never control device execution.
        return event

    def on_event(self, callback):
        self._event_callbacks.append(callback)

    def on_command(self, callback):
        self._command_callbacks.append(callback)

    def renew(self, producer, token, ttl_ns=500_000_000):
        with self.lock:
            now = self._now()
            if self.status != 'ACTIVE' or not self.lease or now >= self.lease['expires_ns']:
                raise RuntimeErrorCode('NO_ACTIVE_AUTHORITY')
            if producer != self.lease['producer'] or token != self.lease['token']:
                raise RuntimeErrorCode('INVALID_AUTHORITY')
            if not isinstance(ttl_ns, int) or not 0 < ttl_ns <= 5_000_000_000:
                raise RuntimeErrorCode('INVALID_LEASE')
            self.lease['expires_ns'] = now + ttl_ns
            return copy.deepcopy(self.lease)

    def revoke(self, producer=None, resources=None, reason='producer_release'):
        with self.lock:
            if producer is not None and self.lease and self.lease['producer'] != producer:
                raise RuntimeErrorCode('INVALID_PRODUCER')
            if self.status in ('FAULTED', 'ESTOP_LATCHED', 'STOPPED'):
                raise RuntimeErrorCode('RESET_REQUIRED')
            errors = self._group_stop(reason)
            if not errors:
                self.status = 'READY_DISARMED'
            return self.state()

    disarm = revoke

    def estop(self, reason='operator_estop'):
        with self.lock:
            self._group_stop(reason)
            self.estop_latched = True
            self.status = 'ESTOP_LATCHED'
            return self.state()

    def reset(self, request_id):
        with self.lock:
            def action():
                if self.status not in ('FAULTED', 'ESTOP_LATCHED'):
                    raise RuntimeErrorCode('RESET_REQUIRES_FAULT')
                if not self.resources.handles:
                    raise RuntimeErrorCode('RESTART_REQUIRED')
                if not all(x['ready'] for x in self._health().values()):
                    raise RuntimeErrorCode('STALE_FEEDBACK')
                errors = self._stop_adapters(self._actuators())
                if errors:
                    self._event('reset_blocked', stop_failures=errors)
                    raise RuntimeErrorCode('STOP_UNCONFIRMED')
                self._revoke()
                self.estop_latched = False
                self.status = 'READY_DISARMED'
                return {'reset': True}
            return self._operation(request_id, 'reset', action)

    def _revoke(self):
        self.epoch += 1
        self.lease = None
        self.sequence.clear()

    def _operation(self, request_id, action, callback):
        if not request_id or not isinstance(request_id, str):
            raise RuntimeErrorCode('REQUEST_ID_REQUIRED')
        if request_id in self.operations:
            old = self.operations[request_id]
            if old['action'] != action:
                raise RuntimeErrorCode('REQUEST_ID_REUSED')
            return copy.deepcopy(old)
        operation = {'request_id': request_id, 'action': action, 'status': 'RUNNING'}
        self.operations[request_id] = operation
        try:
            operation['result'] = callback()
            operation['status'] = 'SUCCEEDED'
        except Exception as exc:
            operation.update(status='FAILED', error=str(exc))
            self._event('operation_failed', action=action, reason=str(exc))
        return copy.deepcopy(operation)

    def _actuators(self):
        return [key for key, value in self.instances.items() if value.get('kind') == 'actuator']

    def _health(self):
        now = self._now()
        result = {}
        for key, adapter in self.adapters.items():
            try:
                value = dict(adapter.health(now))
            except Exception as exc:
                value = {'ready': False, 'sample_ns': 0, 'fault': True, 'reason': 'HEALTH_FAILED: ' + str(exc),
                         'source': 'unavailable', 'measured': False, 'synthetic': False}
            stamp = value.get('sample_ns')
            if type(stamp) is not int or stamp < 0:
                value.update(ready=False, fault=True, reason='INVALID_HEALTH_TIMESTAMP')
                stamp = 0
            age = self._now() - stamp
            value['age_ns'] = age
            value['fresh'] = 0 <= age <= self.max_age_ns
            value['ready'] = value.get('ready') is True and value['fresh']
            if age < 0:
                value.update(fault=True, reason='FUTURE_HEALTH_TIMESTAMP')
            result[key] = value
        return result

    def preflight(self):
        import importlib.util
        import shutil
        checks = []
        def module_missing(name):
            try:
                return importlib.util.find_spec(name) is None
            except (ModuleNotFoundError, ValueError):
                return True
        for key, instance in self.instances.items():
            backend, plugin = instance.get('backend', 'fake'), instance['plugin_id']
            missing = []
            if backend == 'sim':
                if module_missing('mujoco'): missing.append('mujoco')
                if plugin.startswith('morla.camera') and module_missing('PIL'): missing.append('Pillow')
            elif backend == 'real' and plugin.startswith('morla.camera'):
                params = instance.get('parameters', {})
                kind = 'uvc' if plugin.endswith('.uvc') else 'ros_image' if plugin.endswith('.ros_image') else params.get('camera_backend')
                if kind == 'uvc':
                    if module_missing('cv2'): missing.append('opencv')
                    if not params.get('device_path', '').startswith('/dev/v4l/by-id/'): missing.append('stable_camera_device_path')
                elif kind == 'ros_image':
                    if module_missing('rclpy'): missing.append('rclpy')
                    if not params.get('compressed') and module_missing('PIL'): missing.append('Pillow')
                    if not params.get('image_topic'): missing.append('explicit_image_topic')
                else:
                    missing.append('camera_backend')
            elif backend == 'real' and instance.get('kind') == 'input':
                if module_missing('rclpy'): missing.append('rclpy')
                missing_probe = 'external_input_host_and_guarded_source_required'
                checks.append({'instance_id': key, 'backend': backend,
                               'backend_available': not missing, 'missing': missing,
                               'deployment_verified': bool(instance.get('deployment_verified')),
                               'probe': missing_probe, 'render_context_verified': None})
                continue
            elif backend == 'real':
                # External adapters may use a direct TCP/serial SDK without ROS.
                # Inspect installed metadata only; preflight must not load device code.
                from importlib import metadata
                entries = metadata.entry_points(group='morla.adapters')
                selected = [entry for entry in entries if entry.name == plugin + ':real']
                if selected:
                    checks.append({'instance_id': key, 'backend': backend,
                                   'backend_available': len(selected) == 1,
                                   'deployment_verified': bool(instance.get('deployment_verified')),
                                   'missing': [] if len(selected) == 1 else ['ambiguous_adapter_entry_point'],
                                   'probe': 'installed_adapter_metadata_only_dependencies_unchecked',
                                   'render_context_verified': None})
                    continue
                if module_missing('rclpy'): missing.append('rclpy')
                if shutil.which('ros2') is None: missing.append('ros2')
                try:
                    from ament_index_python.packages import get_package_prefix
                    package = {'morla.nero.arm': 'nero_robot_control', 'morla.astral.arm': 'astral_robot_control',
                               'morla.astral.gripper': 'astral_robot_control', 'morla.xhand.hand': 'xhand_adapter',
                               'morla.wuji.hand': 'wujihand_driver'}[plugin]
                    get_package_prefix(package)
                except (ImportError, LookupError, KeyError):
                    missing.append('native_ROS_package_or_installed_adapter')
                vendor = {'morla.nero.arm': 'pyAgxArm', 'morla.astral.arm': 'astral_robot_sdk',
                          'morla.astral.gripper': 'astral_robot_sdk'}.get(plugin)
                if vendor and module_missing(vendor): missing.append(vendor)
            checks.append({'instance_id': key, 'backend': backend, 'backend_available': not missing,
                           'deployment_verified': bool(instance.get('deployment_verified')), 'missing': missing,
                           'probe': 'metadata_only_no_hardware',
                           'render_context_verified': False if backend == 'sim' and plugin.startswith('morla.camera') else None})
        return {'components': checks, 'ready': all(item['backend_available'] for item in checks)}

    def start(self):
        with self.lock:
            if self.status != 'VALIDATED':
                raise RuntimeErrorCode('INVALID_START_STATE')
            self.status = 'PREPARING'
            try:
                keys = ['morla:active-system']
                for session in self.resolved.get('device_sessions', []):
                    keys.append(session.get('resource_key', session['session_id']))
                self.resources.acquire(keys)
                from .plugins import load_installed_adapter
                shared_devices = {}
                for key, instance in self.instances.items():
                    if key in self.adapters:
                        continue
                    context = {'session_id': self.session_id, 'clock': self.clock,
                               'resource_root': str(self.resources.root), 'lane_fd': self.resources.lane_fd,
                               'shared_device_sessions': shared_devices,
                               'device_session': next((copy.deepcopy(device) for device in self.resolved.get('device_sessions', [])
                                                       if key in device.get('instances', [])), None)}
                    adapter = load_installed_adapter(copy.deepcopy(instance), context)
                    if adapter is not None:
                        self.adapters[key] = adapter
                if any(row.get('backend') == 'sim' for row in self.instances.values()):
                    from .sim_mujoco import make_sim_adapters
                    for key, adapter in make_sim_adapters(self).items():
                        self.adapters.setdefault(key, adapter)
                if any(row.get('backend') == 'real' for row in self.instances.values()):
                    from .native_ros import make_native_adapters
                    from .cameras import make_camera_adapters
                    candidates = {**make_native_adapters(self), **make_camera_adapters(self)}
                    for key, adapter in candidates.items():
                        self.adapters.setdefault(key, adapter)
                for key, instance in self.instances.items():
                    if key not in self.adapters:
                        if instance.get('backend', 'fake') != 'fake':
                            raise RuntimeErrorCode('BACKEND_UNAVAILABLE: ' + key)
                        factory = Ros2ControlFakeAdapter if instance['plugin_id'] == 'morla.ros2_control.mock' else FakeAdapter
                        self.adapters[key] = factory(instance)
                    self.adapters[key].configure({'session_id': self.session_id, 'instance': copy.deepcopy(instance)})
                health = self._health()
                if all(x['ready'] for x in health.values()):
                    self.status = 'READY_DISARMED'
                    self._event('ready_disarmed')
                else:
                    self.status = 'PREPARING'
                    self._event('waiting_measured_feedback')
            except Exception:
                self._revoke()
                self._cleanup()
                self.status = 'FAULTED'
                raise
            return self.state()

    def _managed_operation(self, name, request_id, component_ids=None):
        targets = list(component_ids) if component_ids is not None else self._actuators()
        if (not targets and name not in ('prepare', 'activate')) or len(targets) != len(set(targets)) or not set(targets) <= set(self._actuators()):
            raise RuntimeErrorCode('INVALID_OPERATION_RESOURCES')
        signature = (name, tuple(targets))
        with self.lock:
            if not isinstance(request_id, str) or not request_id:
                raise RuntimeErrorCode('REQUEST_ID_REQUIRED')
            old = self.operations.get(request_id)
            if old is not None:
                if old.get('signature') != signature:
                    raise RuntimeErrorCode('REQUEST_ID_REUSED')
                return copy.deepcopy(old)
            if self._pending_operation is not None:
                raise RuntimeErrorCode('OPERATION_BUSY')
            if name == 'activate' and self.status != 'READY_DISARMED':
                raise RuntimeErrorCode('ACTIVATE_REQUIRES_DISARMED')
            if name != 'activate' and self.status not in ('READY_DISARMED', 'PREPARING'):
                raise RuntimeErrorCode('OPERATION_REQUIRES_DISARMED')
            for key in targets:
                if name not in ('activate', 'prepare') and name not in self.instances[key].get('operations', {}):
                    raise RuntimeErrorCode('UNSUPPORTED_OPERATION: ' + key + ':' + name)
                if self.instances[key].get('backend', 'fake') == 'real' and not (
                    getattr(self.adapters[key], 'watchdog_verified', False) and
                    getattr(self.adapters[key], 'deployment_verified', False)):
                    raise RuntimeErrorCode('REAL_DEPLOYMENT_UNVERIFIED: ' + key)
            if name == 'activate' and not all(x['ready'] for x in self._health().values()):
                raise RuntimeErrorCode('STALE_FEEDBACK')
            generation = self._operation_generation
            operation = {'request_id': request_id, 'action': name, 'signature': signature, 'status': 'RUNNING'}
            self.operations[request_id] = operation
            self._pending_operation = request_id

        def check_generation():
            with self.lock:
                if generation != self._operation_generation or self.status in ('STOPPING', 'STOPPED', 'FAULTED', 'ESTOP_LATCHED'):
                    raise RuntimeErrorCode('OPERATION_CANCELLED')

        def execute():
            completed = []
            dispatch_only = []
            try:
                for key in targets:
                    check_generation()
                    adapter = self.adapters[key]
                    if name == 'activate':
                        result = adapter.activate()
                    elif name == 'prepare':
                        result = adapter.prepare()
                    else:
                        result = adapter.operate(name)
                    if result is False:
                        raise RuntimeErrorCode('OPERATION_FAILED: ' + key)
                    if isinstance(result, dict) and result.get('completed') is False:
                        dispatch_only.append({'component_id': key, **result})
                    else:
                        completed.append(key)
                    check_generation()
                with self.lock:
                    check_generation()
                    if name == 'activate':
                        self.status = 'ACTIVE'
                        self._event('activated')
                    elif all(x['ready'] for x in self._health().values()):
                        self.status = 'READY_DISARMED'
                    operation.update(status='DISPATCHED' if dispatch_only else 'SUCCEEDED',
                                     result={'completed': completed, 'pending_confirmation': dispatch_only})
            except Exception as exc:
                # Compensate even a late enable reply after cancellation. Do not
                # hold the control-plane lock while waiting on native services.
                errors = self._stop_adapters(targets)
                with self.lock:
                    self._revoke()
                    if generation == self._operation_generation:
                        self.status = 'FAULTED'
                    operation.update(status='CANCELLED' if str(exc) == 'OPERATION_CANCELLED' else 'FAILED',
                                     error=str(exc), stop_failures=errors, completed=completed)
                    self._event('operation_failed', action=name, reason=str(exc), stop_failures=errors)
            finally:
                with self.lock:
                    if self._pending_operation == request_id:
                        self._pending_operation = None
        asynchronous = any(self.instances[key].get('backend', 'fake') != 'fake' for key in targets)
        if asynchronous:
            threading.Thread(target=execute, name='morla-operation-' + name, daemon=True).start()
        else:
            execute()
        with self.lock:
            return copy.deepcopy(operation)

    def prepare(self, request_id, component_ids=None):
        return self._managed_operation('prepare', request_id, component_ids)

    def activate(self, request_id):
        return self._managed_operation('activate', request_id)

    def operation(self, request_id):
        with self.lock:
            return copy.deepcopy(self.operations[request_id])

    def cancel_operation(self, request_id):
        with self.lock:
            operation = self.operations[request_id]
            if operation['status'] not in ('RUNNING', 'DISPATCHED', 'CANCEL_REQUESTED'):
                return copy.deepcopy(operation)
            errors = self._group_stop('operation_cancelled')
            if not errors:
                self.status = 'READY_DISARMED'
            if operation['status'] in ('RUNNING', 'CANCEL_REQUESTED'):
                operation['status'] = 'CANCEL_REQUESTED'
            else:
                operation['status'] = 'CANCELLED'
            operation['stop_failures'] = errors
            return copy.deepcopy(operation)

    def acquire(self, producer, resources=None, ttl_ns=500_000_000, request_id=None):
        with self.lock:
            if self.status != 'ACTIVE':
                raise RuntimeErrorCode('NOT_ACTIVE')
            now = self._now()
            if self.lease and now >= self.lease['expires_ns']:
                self._group_stop('lease_expired')
                raise RuntimeErrorCode('LEASE_EXPIRED_REARM_REQUIRED')
            if self.lease and now < self.lease['expires_ns']:
                raise RuntimeErrorCode('OWNER_BUSY')
            selected = list(resources) if resources is not None else self._actuators()
            if not selected or len(selected) != len(set(selected)) or not set(selected) <= set(self._actuators()):
                raise RuntimeErrorCode('INVALID_RESOURCES')
            # A declared manipulation group is acquired as a whole.
            for group in self.resolved.get('control_groups', []):
                members = set(group.get('members', []))
                if members.intersection(selected) and not members <= set(selected):
                    raise RuntimeErrorCode('PARTIAL_GROUP_AUTHORITY')
            if not isinstance(producer, str) or not producer or not isinstance(ttl_ns, int) or not 0 < ttl_ns <= 5_000_000_000:
                raise RuntimeErrorCode('INVALID_LEASE')
            self._revoke()
            self.lease = {'session_id': self.session_id, 'producer': producer, 'resources': selected,
                          'epoch': self.epoch, 'token': secrets.token_hex(24), 'expires_ns': now + ttl_ns}
            self._event('authority_granted', producer=producer, resources=selected, epoch=self.epoch)
            return copy.deepcopy(self.lease)

    def switch(self, producer, resources=None, input_received_ns=None, ttl_ns=500_000_000, request_id=None):
        with self.lock:
            if self.status != 'ACTIVE':
                raise RuntimeErrorCode('SWITCH_REQUIRES_ACTIVE')
            self._revoke()
            # All old output is stopped before any grant. A failed ACK leaves FAULTED.
            errors = self._group_stop('authority_switch')
            if errors:
                raise RuntimeErrorCode('STOP_ACK_MISSING')
            now = self._now()
            if input_received_ns is None or not 0 <= now - input_received_ns <= self.max_age_ns:
                self.status = 'READY_DISARMED'
                raise RuntimeErrorCode('INPUT_STALE')
            try:
                for key in self._actuators():
                    if self.adapters[key].activate() is False:
                        raise RuntimeErrorCode('REANCHOR_FAILED: ' + key)
            except Exception:
                self._group_stop('reanchor_failed')
                raise
            self.status = 'ACTIVE'
            return self.acquire(producer, resources, ttl_ns, request_id)

    def submit(self, command):
        with self.lock:
            dispatched = []
            attempted = False
            try:
                now = self._now()
                lease = self.lease
                if self.status != 'ACTIVE' or not lease or now >= lease['expires_ns']:
                    raise RuntimeErrorCode('NO_ACTIVE_AUTHORITY')
                for field in ('session_id', 'producer', 'token', 'epoch'):
                    if command.get(field) != lease[field]:
                        raise RuntimeErrorCode('INVALID_' + field.upper())
                seq = command.get('sequence')
                if not isinstance(seq, int) or isinstance(seq, bool) or seq <= self.sequence.get(lease['epoch'], -1):
                    raise RuntimeErrorCode('STALE_SEQUENCE')
                issued, ttl = command.get('issued_ns'), command.get('ttl_ns')
                if not isinstance(issued, int) or not isinstance(ttl, int) or not 0 < ttl <= 5_000_000_000 or issued > now or now - issued >= ttl:
                    raise RuntimeErrorCode('COMMAND_EXPIRED_OR_FUTURE')
                commands = command.get('commands', [])
                ids = [x['component_id'] for x in commands]
                if not ids or len(ids) != len(set(ids)) or not set(ids) <= set(lease['resources']):
                    raise RuntimeErrorCode('COMMAND_RESOURCES')
                health = self._health()
                for item in commands:
                    key = item['component_id']
                    if not health[key]['ready']:
                        raise RuntimeErrorCode('STALE_FEEDBACK: ' + key)
                    self._validate_target(item, self.instances[key])
                self.sequence[lease['epoch']] = seq
                for item in commands:
                    attempted = True
                    result = self.adapters[item['component_id']].submit(copy.deepcopy(item))
                    if result is False or isinstance(result, dict) and not result.get('accepted', True):
                        raise RuntimeErrorCode('DISPATCH_FAILED: ' + item['component_id'])
                    dispatched.append(item['component_id'])
                receipt = {'accepted': True, 'sequence': seq, 'dispatched': dispatched, 'monotonic_ns': now}
                self._event('command_dispatched', sequence=seq, components=dispatched)
                for callback in self._command_callbacks:
                    try:
                        callback(copy.deepcopy(command), copy.deepcopy(receipt))
                    except Exception as exc:
                        self._event('observer_failed', reason=str(exc))
                return receipt
            except Exception as exc:
                if attempted:
                    self._group_stop('partial_dispatch')
                self._event('command_rejected', reason=str(exc), dispatched=dispatched)
                return {'accepted': False, 'reason': str(exc), 'dispatched': dispatched}

    @staticmethod
    def _validate_target(item, instance):
        joints = instance.get('joints', [])
        names = [x['name'] if isinstance(x, dict) else x for x in joints]
        if item.get('names') != names or len(item.get('values', [])) != len(names):
            raise RuntimeErrorCode('JOINT_SET_MISMATCH')
        mode = item.get('mode', 'position')
        if mode not in instance.get('command_modes', ['position']):
            raise RuntimeErrorCode('UNSUPPORTED_MODE')
        units = [x.get('unit', 'rad') if isinstance(x, dict) else 'rad' for x in joints]
        if item.get('units') != units:
            raise RuntimeErrorCode('UNIT_MISMATCH')
        for value, joint in zip(item['values'], joints):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
                raise RuntimeErrorCode('NONFINITE_TARGET')
            if isinstance(joint, dict):
                limits = joint.get('limits', {})
                if isinstance(limits, (list, tuple)) and len(limits) == 2:
                    limits = {'lower': limits[0], 'upper': limits[1]}
                if not isinstance(limits, dict):
                    raise RuntimeErrorCode('INVALID_JOINT_LIMIT_CONTRACT')
                low, high = joint.get('lower', limits.get('lower')), joint.get('upper', limits.get('upper'))
                if low is not None and value < low or high is not None and value > high:
                    raise RuntimeErrorCode('JOINT_LIMIT')

    def tick(self):
        with self.lock:
            now = self._now()
            if self.status == 'PREPARING' and any(x.get('fault') for x in self._health().values()):
                self._group_stop('component_startup_fault')
            if self.status == 'PREPARING' and all(x['ready'] for x in self._health().values()):
                self.status = 'READY_DISARMED'
                self._event('ready_disarmed')
            if self.status == 'ACTIVE':
                if self.lease and now >= self.lease['expires_ns']:
                    self._group_stop('lease_expired')
                elif not all(x['ready'] for x in self._health().values()):
                    self._group_stop('feedback_stale')
            return self.state()

    def _stop_adapters(self, targets, timeout_s=1.0):
        # Independent devices stop concurrently. A hanging injected adapter cannot
        # hold the control plane forever. Real SDK state remains in owned child
        # processes and must have its separately verified board watchdog.
        jobs, errors = [], []
        grouped = {}
        for key in targets:
            adapter = self.adapters[key]
            physical = id(adapter.session) if getattr(adapter, 'device_scoped_stop', False) else id(adapter)
            grouped.setdefault(physical, []).append(key)
        for keys in grouped.values():
            done, outcome = threading.Event(), {}
            def stop_one(keys=keys, done=done, outcome=outcome):
                try:
                    if self.adapters[keys[0]].stop() is False:
                        raise RuntimeErrorCode('STOP_ACK_MISSING')
                    outcome['ok'] = True
                except Exception as exc:
                    outcome['error'] = str(exc)
                finally:
                    done.set()
            threading.Thread(target=stop_one, name='morla-stop-' + keys[0], daemon=True).start()
            jobs.append((keys, done, outcome))
        deadline = time.monotonic() + timeout_s
        for keys, done, outcome in jobs:
            if not done.wait(max(0.0, deadline - time.monotonic())):
                errors.append({'component': keys[0], 'reason': 'STOP_TIMEOUT'})
            elif not outcome.get('ok'):
                errors.append({'component': keys[0], 'reason': outcome.get('error', 'STOP_FAILED')})
            else:
                for key in keys:
                    if hasattr(self.adapters[key], 'active'):
                        self.adapters[key].active = False
        return errors

    def _group_stop(self, reason):
        self._operation_generation += 1
        self._revoke()
        errors = self._stop_adapters(self._actuators())
        self.status = 'FAULTED' if errors or reason not in ('lease_expired',) else 'READY_DISARMED'
        self._event('group_stopped', reason=reason, errors=errors)
        return errors

    def _cleanup(self):
        available = [key for key in self._actuators() if key in self.adapters]
        errors = self._stop_adapters(available)
        jobs = []
        grouped = {}
        for key, adapter in self.adapters.items():
            grouped.setdefault(id(getattr(adapter, 'session', adapter)), (key, adapter))
        for key, adapter in reversed(list(grouped.values())):
            done, outcome = threading.Event(), {}
            def close_one(adapter=adapter, done=done, outcome=outcome):
                try:
                    if adapter.close() is False:
                        raise RuntimeErrorCode('CLOSE_FAILED')
                    outcome['ok'] = True
                except Exception as exc:
                    outcome['error'] = str(exc)
                finally:
                    done.set()
            threading.Thread(target=close_one, name='morla-close-' + key, daemon=True).start()
            jobs.append((key, done, outcome))
        deadline = time.monotonic() + 3.0
        close_uncertain = False
        for key, done, outcome in jobs:
            if not done.wait(max(0, deadline - time.monotonic())) or not outcome.get('ok'):
                close_uncertain = True
                failure = {'component': key, 'reason': outcome.get('error', 'CLOSE_TIMEOUT')}
                errors.append(failure)
                self._event('close_failed', **failure)
        # Unknown cleanup cannot free the host lane for a second device owner.
        # Owned children also retain the inherited lock until they actually exit.
        if not close_uncertain:
            self.resources.release()
        return errors

    def stop(self, request_id=None):
        with self.lock:
            def action():
                self.status = 'STOPPING'
                errors = self._group_stop('operator_stop')
                errors.extend(self._cleanup())
                self.status = 'FAULTED' if errors else 'STOPPED'
                return {'errors': errors}
            return self._operation(request_id or uuid.uuid4().hex, 'stop', action)

    def invoke(self, operation, request_id, component_ids=None):
        if operation == 'status':
            return self.operation(request_id)
        if operation == 'cancel':
            return self.cancel_operation(request_id)
        if operation in ('home', 'workpos', 'disable'):
            return self._managed_operation(operation, request_id, component_ids)
        if operation == 'reset':
            return self.reset(request_id)
        if operation == 'estop':
            return self.estop()
        if operation == 'prepare':
            return self.prepare(request_id, component_ids)
        if operation == 'activate':
            return self.activate(request_id)
        if operation == 'stop':
            return self.stop(request_id)
        raise RuntimeErrorCode('UNSUPPORTED_OPERATION')

    def state(self):
        with self.lock:
            return {'session_id': self.session_id, 'status': self.status, 'state': self.status,
                'epoch': self.epoch, 'authority': None if not self.lease else {
                    k: v for k, v in self.lease.items() if k != 'token'},
                'components': self._health() if self.status not in ('STOPPED', 'FAULTED', 'ESTOP_LATCHED') else {},
                'events': copy.deepcopy(self.events[-100:]), 'operations': copy.deepcopy(list(self.operations.values()))}
