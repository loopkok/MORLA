import copy
from types import SimpleNamespace
import numpy as np
import pytest
from morla_core.composition import compile_system
from morla_teleop.controllers import ControllerRegistry
from morla_teleop.retargeting import XHandRetargetController,WujiRetargetController,install_controller_plugin,controller_availability,RetargetingUnavailable


def instance(plugin,side='left'):
    return compile_system({'assembly_version':1,'assembly_id':'retarget','revision':1,'instances':{'hand':{'plugin':plugin,'parameters':{'side':side,'enable_thumb_fix':False}}}})['instances'][0]


def landmarks():
    rng=np.random.default_rng(91)
    return rng.normal(size=(21,3))*.02


class DexStub:
    def __init__(self,names):
        self.joint_names=list(reversed(names));self.calls=[];self.resets=0
        self.optimizer=SimpleNamespace(target_link_human_indices=np.array([[0,0],[4,8]]),retargeting_type='VECTOR')
    def retarget(self,references):
        self.calls.append(references.copy());return np.arange(len(self.joint_names),dtype=float)/100
    def reset(self):self.resets+=1


@pytest.mark.parametrize('side',['left','right'])
def test_xhand_reuses_legacy_preprocess_named_reorder_and_sign(side):
    from xhand_retargeting.preprocess import preprocess_xhand_landmarks
    hand=instance('morla.xhand.hand',side);names=[joint['name'] for joint in hand['joints']]
    stub=DexStub([side+'_hand_'+name for name in names]);controller=XHandRetargetController(hand,stub)
    raw=landmarks();before=raw.copy();output=controller(raw,[0]*12,.01,{})
    expected_points=preprocess_xhand_landmarks(raw,side,'legacy_xhand_pinky')
    np.testing.assert_allclose(stub.calls[0],expected_points[[4,8]]-expected_points[[0,0]])
    np.testing.assert_array_equal(raw,before)
    expected=dict(zip(stub.joint_names,np.arange(12)/100))
    assert output[0]==pytest.approx(expected[side+'_hand_'+names[0]])
    assert output[3]==pytest.approx(expected[side+'_hand_'+names[3]]*(-1 if side=='right' else 1))
    controller.reset();assert stub.resets==1
    with pytest.raises(ValueError):controller(np.zeros((20,3)),[0]*12,.01,{})
    with pytest.raises(ValueError):controller(np.full((21,3),np.nan),[0]*12,.01,{})


def test_wuji_named_mapping_limits_no_scalar_expansion():
    hand=instance('morla.wuji.hand');hand['joints'].reverse()
    backend=SimpleNamespace(retarget=lambda points:np.arange(20,dtype=float)/100,reset=lambda:None)
    controller=WujiRetargetController(hand,backend)
    result=controller(landmarks(),[0]*20,.01,{})
    assert len(result)==20
    # Descriptor is reversed, bridge uses stable finger-joint names.
    limits=hand['joints'][0]['limits'];assert result[0]==pytest.approx(np.clip(.19,*limits))
    backend.retarget=lambda points:np.array([.5])
    with pytest.raises(ValueError,match='all 20'):controller(landmarks(),[0]*20,.01,{})


def test_selected_controller_plugin_only_loads_selected(monkeypatch):
    import importlib.metadata
    loaded=[]
    class Entry:
        def __init__(self,name):self.name=name
        def load(self):
            loaded.append(self.name)
            return lambda context:lambda value,measured,dt,parameters:[context['value']]
    monkeypatch.setattr(importlib.metadata,'entry_points',lambda **kwargs:[Entry('lab.selected'),Entry('lab.other')])
    registry=ControllerRegistry();install_controller_plugin(registry,'lab.selected',{'value':.2})
    assert loaded==['lab.selected']
    assert registry.apply('lab.selected',None,[0],.01,{})==[.2]
    with pytest.raises(RetargetingUnavailable):install_controller_plugin(registry,'unknown')
    assert loaded==['lab.selected']

def test_missing_optional_dependency_preflight_is_explicit(monkeypatch):
    import importlib.util
    monkeypatch.setattr(importlib.util,'find_spec',lambda name:None)
    report=controller_availability('morla.xhand.retarget',instance('morla.xhand.hand'))
    assert report['available'] is False
    assert 'dex_retargeting' in report['missing_dependencies']
    assert report['configuration_probe_performed'] is False
