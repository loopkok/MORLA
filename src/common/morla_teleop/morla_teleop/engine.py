"""Transport-neutral binding execution shared by Web, ROS and embedded callers."""
from morla_core.cadence_probe import record as cadence_record
import logging
import time

from .controllers import BindingController, ControllerRegistry

_log = logging.getLogger(__name__)


class BindingEngine:
    def __init__(self, runtime, *, bindings=None, operator_hold=None):
        self.runtime = runtime
        self.clock = runtime.clock
        self.bindings = bindings if bindings is not None else {}
        self.deadman = {}
        self.basis_warnings = {}
        self.operator_hold = operator_hold

    def reserves_input_action(self, input_id, action):
        return action in ('record_or_release', 'record_stop') and any(
            binding['source'] == input_id + '.right_ab'
            for binding in self.runtime.resolved.get('bindings', ())
        )

    def _trace(self, kind, data, **context):
        try:
            trace = getattr(self.runtime, 'trace', None)
            if trace is not None: trace(kind, data, **context)
        except Exception: pass

    def _trace_enabled(self, kind, **scope):
        enabled = getattr(self.runtime, 'trace_enabled', None)
        if enabled is not None:
            return enabled(kind, **scope)
        return getattr(self.runtime, 'trace', None) is not None

    @staticmethod
    def _reports(controller):
        reports = {}
        for name, plugin in controller.registry.plugins.items():
            try:
                diagnostic = getattr(plugin, 'diagnostics', None)
                if callable(diagnostic): reports[name] = diagnostic()
                elif hasattr(plugin, 'last_report'): reports[name] = plugin.last_report
            except Exception as exc: reports[name] = {'diagnostic_error':str(exc)}
        return reports

    def _binding_controller(self, binding):
        controller=self.bindings.get(binding['id'])
        if controller is not None:return controller
        target=binding['target'].split('.',1)[0]
        instance=self.runtime.instances[target]
        adapter=self.runtime.adapters[target]
        names=[j['name'] if isinstance(j,dict) else j for j in instance['joints']]
        age=binding.get('max_age_ms',250)
        if isinstance(age,bool) or not isinstance(age,(int,float)) or age<=0:
            raise ValueError('binding max_age_ms must be positive')
        registry=ControllerRegistry()
        if binding['controller'] in {'morla.pinch_filtered','morla.trigger_filtered'}:
            from .gripper_processing import FilteredGripperController
            registry.register(binding['controller'],FilteredGripperController(binding.get('parameters',{}),
                landmarks=binding['controller']=='morla.pinch_filtered',clock=self.clock))
        elif binding['controller'] in {'morla.cartesian_delta_ik','morla.gamepad_cartesian_jog'}:
            from morla_teleop.kinematics import install_kinematics
            install_kinematics(registry,instance,solver=getattr(adapter,'kinematics_provider',None))
        elif binding['controller'] in {'morla.xhand.retarget','morla.wuji.retarget'}:
            from morla_teleop.retargeting import install_retargeting
            install_retargeting(registry,instance)
        elif binding['controller']=='morla.scalar_anchored':
            from .controllers import AnchoredScalarController
            registry.register(binding['controller'],AnchoredScalarController())
        elif binding['controller'] not in {'morla.joint_jog','morla.scalar_map','morla.identity','morla.pinch_scalar'}:
            from morla_teleop.retargeting import install_controller_plugin
            install_controller_plugin(registry,binding['controller'],context={
                'instance':instance,'binding':binding,'clock':self.clock,
                'kinematics_provider':getattr(adapter,'kinematics_provider',None)})
        controller=BindingController(binding,names,registry=registry,clock=self.clock,
                                     max_age_ns=int(age*1_000_000))
        if self._trace_enabled("controller"):
            for plugin_id, plugin in registry.plugins.items():
                setter = getattr(plugin, 'set_diagnostic_sink', None)
                if setter is not None:
                    def sink(report, plugin_id=plugin_id):
                        self._trace('controller', {'plugin_id':plugin_id, 'report':report},
                                    binding_id=binding['id'], component_id=target)
                    try: setter(sink)
                    except Exception as exc: self._trace('diagnostic_hook_failed', {'reason':str(exc)}, binding_id=binding['id'])
        self.bindings[binding['id']]=controller
        return controller

    def targets(self, frame):
        try: return self._targets(frame)
        except Exception as exc:
            self._trace('control_pipeline_error', {'source_id':getattr(frame,'source_id',None),
                        'sequence':getattr(frame,'sequence',None),'reason':str(exc)})
            raise

    def _targets(self, frame):
        trace_input=getattr(self.runtime,'trace_input',None)
        if trace_input is not None: trace_input(frame)
        # Judge freshness against the moment the frame entered the pipeline. A
        # binding that lazily builds its controller (a MuJoCo model, a URDF
        # solver) can take hundreds of ms on the first frame; reading the clock
        # per binding would then age the same frame into staleness and drop the
        # remaining arms for no reason the operator can act on.
        now = self.clock()
        targets = []
        for binding in self.runtime.resolved.get('bindings', []):
            if binding['source'].split('.', 1)[0] != frame.source_id:
                continue
            source_port=binding['source'].split('.',1)[1]
            if source_port not in frame.ports:
                self._release_binding(binding)
                self._trace('control_gate', {'reason':'source_port_missing','input_sequence':frame.sequence}, binding_id=binding['id'])
                continue
            if not self._input_basis_matches(binding, frame):
                self._release_binding(binding)
                self._trace('control_gate', {'reason':'input_basis_mismatch','input_sequence':frame.sequence}, binding_id=binding['id'])
                continue
            if not self._deadman_held(binding, frame, now):
                self._trace('control_gate', {'reason':'deadman_released_or_stale','input_sequence':frame.sequence}, binding_id=binding['id'])
                continue
            key = binding['target'].split('.', 1)[0]
            instance = self.runtime.instances[key]
            feedback = self.runtime.adapters[key].health(now)
            measured = feedback.get('values')
            if measured is None:
                raise ValueError('adapter does not expose measured positions: ' + key)
            controller=self._binding_controller(binding)
            if self._release_on_input_loss(controller) and not controller.input_fresh(frame):
                self._release_binding(binding)
                continue
            port=binding['source'].split('.',1)[1]
            metadata=(frame.port_metadata or {}).get(port,{})
            sample_key=(frame.source_epoch,metadata.get('received_ns'),metadata.get('frame_id',frame.frame_id),
                        metadata.get('sample_sequence'))
            if ('received_ns' in metadata and frame.valid and controller.input_fresh(frame)
                    and sample_key==getattr(controller,'_last_port_sample',None)):
                continue
            detailed = self._trace_enabled('control', binding_id=binding['id'], component_id=key)
            if detailed and self._trace_enabled('controller'):
                context={'source_id':frame.source_id,'source_epoch':frame.source_epoch,'sequence':frame.sequence,
                         'received_ns':frame.received_ns,'frame_id':getattr(frame,'frame_id',None),'binding_id':binding['id']}
                for plugin in controller.registry.plugins.values():
                    setter=getattr(plugin,'set_trace_context',None)
                    if setter is not None:
                        try: setter(context)
                        except Exception: pass
            if port in ('left_pose', 'right_pose'):
                cadence_record('engine.' + port, counts=('consumed',))
            began=time.perf_counter_ns() if detailed else 0
            try:
                values = controller.step(frame, measured, now)
            except Exception as exc:
                self._trace('control_error', {'reason':str(exc),'input_sequence':frame.sequence,
                            'feedback':feedback,'reports':self._reports(controller)},
                            binding_id=binding['id'], component_id=key)
                raise
            if detailed:
                self._trace('control', {'controller':binding['controller'],'input_sequence':frame.sequence,
                            'input_epoch':frame.source_epoch,'input_received_ns':frame.received_ns,
                            'source_port':port,'input':frame.ports[port], 'feedback':feedback,
                            'target_values':values,'produced_target':values is not None,
                            'dispatch_acknowledged':False,'elapsed_ns':time.perf_counter_ns()-began,
                            'reports':self._reports(controller)}, binding_id=binding['id'], component_id=key)
            controller._last_port_sample=sample_key
            if values is not None:
                if len(values)!=len(instance['joints']):
                    raise ValueError('controller output dimension differs from target: '+key)
                target_port = instance.get('ports', {}).get(binding['target'].split('.', 1)[1], {})
                unit = target_port.get('unit')
                units = ([unit] * len(instance['joints']) if unit and unit != 'mixed'
                         else [j.get('unit', 'rad') for j in instance['joints']])
                targets.append({'component_id': key,
                    'names': [j['name'] for j in instance['joints']], 'values': values,
                    'mode': binding.get('mode', 'velocity' if all(u in {'rad/s','m/s','ratio/s','m_rad/s','sdk_gripper/s'} for u in units) else 'position'),
                    'units': units})
        return targets

    def _input_basis_matches(self, binding, frame):
        """Drop frames whose declared basis differs from what the receiver sent.

        A stale receiver configured for the canonical basis must not feed a
        binding calibrated for raw Unity: the axes are reflected, so motion is
        wrong in a way no freshness check catches. Mismatches are dropped (not
        raised) because they are recoverable by restarting the right receiver.
        Frames without a basis (synthetic/test sources) are trusted.
        """
        declared = binding.get('input_basis_id')
        observed = getattr(frame, 'input_basis_id', None)
        if declared is None or observed is None or declared == observed:
            return True
        now = self.clock()
        last = self.basis_warnings.get(binding['id'])
        if last is None or now - last >= 1_000_000_000:
            _log.warning('dropping %s: input basis %r does not match declared %r',
                         binding['id'], observed, declared)
            self.basis_warnings[binding['id']] = now
        controller = self.bindings.get(binding['id'])
        if controller is not None:
            controller.reset()
        return False

    def _deadman_held(self, binding, frame, now=None):
        """Hold-to-move gate declared by ``deadman``; absent means free-running.

        Absent or stale deadman samples count as released: a dropped grip event
        must stop the arm, not leave it following an old hand pose. Releasing
        also drops the controller anchor, because the hand drifts while the
        grip is open and keeping the old anchor would jump the arm on re-press.

        ``now`` is the frame's entry time; the caller passes it so all bindings
        in one frame agree about how old that frame is.

        The declared deadman is the only physical release for a binding. The
        optional ``operator_hold`` is an explicit client hold (keyboard), never
        a mirror of a hand grip: see the note below.
        """
        source = binding.get('deadman')
        if source is None or source == '':
            return True
        owner, _, port = source.partition('.')
        held = False
        if frame.valid and owner == frame.source_id and port in frame.ports:
            value = frame.ports[port]
            held = value is True or (isinstance(value, (int, float))
                                     and not isinstance(value, bool) and value > .5)
            if held:
                stamp = (frame.port_metadata or {}).get(port, {}).get('received_ns', frame.received_ns)
                now = self.clock() if now is None else now
                held = 0 <= now - stamp <= binding.get('max_age_ms', 250) * 1_000_000
        previous = self.deadman.get(binding['id'], False)
        if self.operator_hold is not None:
            # An explicit operator hold is a deliberate "I am holding the
            # control" from a client that has no physical grip (the keyboard
            # client), so it opens every binding. A hand's grip must never be
            # routed here: each grip is only the deadman its own bindings
            # declare, and one hand opening the gate of the other arm made the
            # untended arm follow its controller while its grip was open.
            held = held or bool(self.operator_hold())
        if not held and previous:
            self._release_binding(binding)
        else:
            self.deadman[binding['id']] = held
        return held

    @staticmethod
    def _release_on_input_loss(controller):
        return any(getattr(plugin, 'release_on_input_loss', False)
                   for plugin in controller.registry.plugins.values())

    def _release_binding(self, binding):
        controller = self.bindings.get(binding['id'])
        release = self.deadman.get(binding['id'], False)
        if controller is not None:
            release |= (getattr(controller, '_last_port_sample', None) is not None
                        and self._release_on_input_loss(controller))
            controller.reset()
        if release:
            self.deadman[binding['id']] = False
            key = binding['target'].split('.', 1)[0]
            release = getattr(self.runtime.adapters[key], 'release_target', None)
            if release is not None:
                release()

    def reset(self):
        self.deadman.clear()
        self.basis_warnings.clear()
        for controller in self.bindings.values(): controller.reset()

    def close(self):
        errors = []
        for controller in self.bindings.values():
            try: controller.close()
            except Exception as exc: errors.append(str(exc))
        self.bindings.clear()
        self.deadman.clear()
        self.basis_warnings.clear()
        return errors
