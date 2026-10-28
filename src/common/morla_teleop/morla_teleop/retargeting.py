"""Selected-only retargeting bridges reuse existing calibrated hand pipelines.

No ROS node, motor, or vendor SDK is imported during catalogue discovery. Building
one of these bridges is an explicit preflight/selection action and may require
installed dex-retargeting or Wuji SDK and installed robot model assets.
"""
from __future__ import annotations
import importlib.metadata
import importlib.util
import numpy as np

class RetargetingUnavailable(RuntimeError):
    pass

_XHAND=('thumb_bend_joint','thumb_rota_joint1','thumb_rota_joint2','index_bend_joint','index_joint1','index_joint2','mid_joint1','mid_joint2','ring_joint1','ring_joint2','pinky_joint1','pinky_joint2')
_WUJI=tuple(f'finger{finger}_joint{joint}' for finger in range(1,6) for joint in range(1,5))


def _landmarks(value):
    points=np.asarray(value,dtype=np.float64)
    if points.shape==(63,):points=points.reshape(21,3)
    if points.shape!=(21,3) or not np.isfinite(points).all():raise ValueError('retargeting requires exactly 21 finite xyz hand landmarks')
    return points.copy()


def _joint_names(instance):
    names=[joint['name'] for joint in instance['joints']]
    if len(names)!=len(set(names)):raise ValueError('ambiguous canonical hand joints')
    return names


def _bounded(instance,names,values):
    values=np.asarray(values,dtype=float)
    if values.shape!=(len(names),) or not np.isfinite(values).all():raise ValueError('retargeting returned invalid joint values')
    joints={joint['name']:joint for joint in instance['joints']}
    limits=[joints[name].get('limits',[joints[name].get('lower'),joints[name].get('upper')]) for name in names]
    if any(len(pair)!=2 or pair[0] is None or pair[1] is None for pair in limits):raise ValueError('retargeting requires explicit per-joint limits')
    return np.clip(values,[pair[0] for pair in limits],[pair[1] for pair in limits]).tolist()


class XHandRetargetController:
    """Quest canonical robot-axis landmarks -> established XHand MANO/DexPilot."""
    def __init__(self,instance,retargeter=None):
        self.instance=instance;self.parameters=instance.get('parameters',{})
        self.side=self.parameters.get('side','left')
        if self.side not in {'left','right'}:raise ValueError('hand side must be explicit left/right')
        self.names=_joint_names(instance)
        if set(self.names)!=set(_XHAND):raise ValueError('XHand canonical joint set differs from established hardware mapping')
        if retargeter is None:
            try:
                import yaml
                from dex_retargeting.retargeting_config import RetargetingConfig
                from morla_core.resource_locator import resolve_package_resource,resolve_robot_asset
                path=resolve_package_resource('xhand_retargeting',f'config/xhand_{self.side}_dexpilot.yml')
                config=yaml.safe_load(path.read_text())['retargeting']
                config['urdf_path']=str(resolve_robot_asset(f'xhand_{self.side}'))
                retargeter=RetargetingConfig.from_dict(config).build()
            except Exception as exc:raise RetargetingUnavailable('XHand retargeting unavailable: '+str(exc)) from exc
        self.retargeter=retargeter
        self._factory=None
        self._needs_reset=False

    def reset(self):
        if hasattr(self.retargeter,'reset'):self.retargeter.reset()
        elif hasattr(self.retargeter,'optimizer'):
            # dex SeqRetargeting retains LP state in last_qpos. Its public reset
            # method is preferred; never carry hidden filters across input epochs.
            self._needs_reset=True

    def __call__(self,value,measured,dt,parameters):
        from xhand_retargeting.preprocess import preprocess_xhand_landmarks
        if self._needs_reset:
            # A selected production controller can be rebuilt deterministically
            # from installed config; injected tests/backends must expose reset.
            fresh=type(self)(self.instance)
            self.retargeter=fresh.retargeter;self._needs_reset=False
        points=preprocess_xhand_landmarks(_landmarks(value),self.side,self.parameters.get('preprocess_mode','legacy_xhand_pinky'))
        optimizer=self.retargeter.optimizer;indices=optimizer.target_link_human_indices
        references=points[indices,:] if optimizer.retargeting_type=='POSITION' else points[indices[1,:],:]-points[indices[0,:],:]
        output=np.asarray(self.retargeter.retarget(references),dtype=float)
        names=list(map(str,self.retargeter.joint_names))
        if output.shape!=(len(names),) or len(set(names))!=len(names) or not np.isfinite(output).all():raise ValueError('XHand backend output/names invalid')
        lookup=dict(zip(names,output));prefix=f'{self.side}_hand_'
        if any(prefix+name not in lookup for name in _XHAND):raise ValueError('XHand backend omitted required named joint')
        result={name:float(lookup[prefix+name]) for name in _XHAND}
        sign=self.parameters.get('index_bend_sign',-1.0 if self.side=='right' else 1.0)
        if sign not in {-1.0,1.0}:raise ValueError('index_bend_sign must be +/-1')
        result['index_bend_joint']*=sign
        if self.parameters.get('enable_thumb_fix',True):
            mcp,ip,tip=points[2],points[3],points[4]
            straightness=float(np.linalg.norm(tip-mcp))/max(float(np.linalg.norm(ip-mcp)+np.linalg.norm(tip-ip)),.001)
            if straightness>.85:result['thumb_rota_joint2']*=1-(straightness-.85)/.15*.3
        return _bounded(self.instance,self.names,[result[name] for name in self.names])


class WujiRetargetController:
    """Reuse selected official/Wuji DexPilot backend and its 5x4 named layout."""
    def __init__(self,instance,backend=None):
        self.instance=instance;self.parameters=instance.get('parameters',{});self.side=self.parameters.get('side','left')
        if self.side not in {'left','right'}:raise ValueError('hand side must be left/right')
        self.names=_joint_names(instance)
        if set(self.names)!=set(_WUJI):raise ValueError('Wuji requires all 20 named joints')
        if backend is None:
            try:
                selected=self.parameters.get('retarget_backend','official')
                if selected=='official':
                    from wujihand_retargeting.backends.official import OfficialRetargetBackend
                    backend=OfficialRetargetBackend(self.side)
                elif selected=='dexpilot':
                    from wujihand_retargeting.backends.dexpilot import DexPilotRetargetBackend
                    backend=DexPilotRetargetBackend(self.side)
                else:raise ValueError('unsupported Wuji retarget backend')
            except Exception as exc:raise RetargetingUnavailable('Wuji retargeting unavailable: '+str(exc)) from exc
        self.backend=backend;self._needs_reset=False
        self._rebuild_on_reset=self.parameters.get('retarget_backend','official')=='dexpilot'
    def reset(self):
        if self._rebuild_on_reset:self._needs_reset=True
        elif hasattr(self.backend,'reset'):self.backend.reset()
        else:self._needs_reset=True
    def __call__(self,value,measured,dt,parameters):
        if self._needs_reset:
            self.backend=type(self)(self.instance).backend;self._needs_reset=False
        output=np.asarray(self.backend.retarget(_landmarks(value)),dtype=float)
        if output.shape!=(20,) or not np.isfinite(output).all():raise ValueError('Wuji backend must return all 20 finite joints')
        lookup=dict(zip(_WUJI,output))
        return _bounded(self.instance,self.names,[lookup[name] for name in self.names])


def install_retargeting(registry,instance):
    plugin=instance['plugin_id']
    if plugin=='morla.xhand.hand':name='morla.xhand.retarget';controller=XHandRetargetController(instance)
    elif plugin=='morla.wuji.hand':name='morla.wuji.retarget';controller=WujiRetargetController(instance)
    else:raise RetargetingUnavailable('No built-in retarget bridge for '+plugin)
    registry.register(name,controller)
    return controller


def install_controller_plugin(registry,plugin_id,context=None):
    """Load exactly one selected installed ``morla.controllers`` entry point."""
    candidates=[entry for entry in importlib.metadata.entry_points(group='morla.controllers') if entry.name==plugin_id]
    if len(candidates)!=1:raise RetargetingUnavailable('Selected controller is missing or ambiguous: '+plugin_id)
    factory=candidates[0].load()  # Selected installed code, never an Assembly import path.
    controller=factory(context or {})
    if not callable(controller):raise RetargetingUnavailable('Installed controller factory did not return callable')
    registry.register(plugin_id,controller)
    return controller


def controller_availability(plugin_id,instance):
    dependencies=[]
    if plugin_id=='morla.xhand.retarget':dependencies=['xhand_retargeting','dex_retargeting','ament_index_python']
    elif plugin_id=='morla.wuji.retarget':dependencies=['wujihand_retargeting','wuji_sdk' if instance.get('parameters',{}).get('retarget_backend','official')=='official' else 'dex_retargeting']
    else:
        matches=[entry for entry in importlib.metadata.entry_points(group='morla.controllers') if entry.name==plugin_id]
        return {'controller':plugin_id,'available':len(matches)==1,'reason':None if len(matches)==1 else 'missing or ambiguous installed controller'}
    missing=[name for name in dependencies if importlib.util.find_spec(name) is None]
    return {'controller':plugin_id,'available':not missing,'missing_dependencies':missing,'configuration_probe_performed':False}
