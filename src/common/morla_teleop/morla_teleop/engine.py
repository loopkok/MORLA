"""Transport-neutral binding execution shared by Web, ROS and embedded callers."""
from .controllers import BindingController, ControllerRegistry


class BindingEngine:
    def __init__(self, runtime, *, bindings=None):
        self.runtime = runtime
        self.clock = runtime.clock
        self.bindings = bindings if bindings is not None else {}

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
        elif binding['controller'] not in {'morla.joint_jog','morla.scalar_map','morla.identity','morla.pinch_scalar'}:
            from morla_teleop.retargeting import install_controller_plugin
            install_controller_plugin(registry,binding['controller'],context={
                'instance':instance,'binding':binding,'clock':self.clock,
                'kinematics_provider':getattr(adapter,'kinematics_provider',None)})
        controller=BindingController(binding,names,registry=registry,clock=self.clock,
                                     max_age_ns=int(age*1_000_000))
        self.bindings[binding['id']]=controller
        return controller

    def targets(self, frame):
        targets = []
        for binding in self.runtime.resolved.get('bindings', []):
            if binding['source'].split('.', 1)[0] != frame.source_id:
                continue
            source_port=binding['source'].split('.',1)[1]
            if source_port not in frame.ports:
                previous=self.bindings.get(binding['id'])
                if previous is not None:previous.reset()
                continue
            key = binding['target'].split('.', 1)[0]
            instance = self.runtime.instances[key]
            measured = self.runtime.adapters[key].health(self.clock()).get('values')
            if measured is None:
                raise ValueError('adapter does not expose measured positions: ' + key)
            controller=self._binding_controller(binding)
            port=binding['source'].split('.',1)[1]
            metadata=(frame.port_metadata or {}).get(port,{})
            sample_key=(frame.source_epoch,metadata.get('received_ns'),metadata.get('frame_id',frame.frame_id))
            if ('received_ns' in metadata and frame.valid and controller.input_fresh(frame)
                    and sample_key==getattr(controller,'_last_port_sample',None)):
                continue
            values = controller.step(frame, measured)
            controller._last_port_sample=sample_key
            if values is not None:
                if len(values)!=len(instance['joints']):
                    raise ValueError('controller output dimension differs from target: '+key)
                targets.append({'component_id': key,
                    'names': [j['name'] for j in instance['joints']], 'values': values,
                    'mode': binding.get('mode', 'position'),
                    'units': [j.get('unit', 'rad') for j in instance['joints']]})
        return targets

    def reset(self):
        for controller in self.bindings.values(): controller.reset()

    def close(self):
        errors = []
        for controller in self.bindings.values():
            try: controller.close()
            except Exception as exc: errors.append(str(exc))
        self.bindings.clear()
        return errors
