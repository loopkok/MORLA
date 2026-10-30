from types import SimpleNamespace
import pytest
from morla_teleop.engine import BindingEngine
from morla_input.adapters.normalized import InputFrame


def runtime(plugin, unit, output, source='input'):
    joint={'name':'opening','unit':unit}
    instance={'instance_id':'gripper','plugin_id':plugin,'joints':[joint]}
    binding={'id':'control','source':source+'.scalar','target':'gripper.position_target',
             'controller':'morla.scalar_map','parameters':{'input_range':[0,1],'output_range':output}}
    return SimpleNamespace(clock=lambda:100,instances={'gripper':instance},
        adapters={'gripper':SimpleNamespace(health=lambda now:{'values':[0.]})},
        resolved={'bindings':[binding]})


@pytest.mark.parametrize('plugin,unit,output,expected',[
    ('vendor.epg','m',[.05,0],.025),('vendor.astral','ratio',[0,1],.5),
    ('unknown.new_gripper','m',[.1,0],.05)])
def test_same_input_different_grippers(plugin,unit,output,expected):
    engine=BindingEngine(runtime(plugin,unit,output))
    frame=InputFrame('input','epoch',0,100,'input',{'scalar':.5})
    targets=engine.targets(frame)
    assert targets[0]['values']==pytest.approx([expected])
    assert targets[0]['units']==[unit]
    engine.close()


@pytest.mark.parametrize('source',['vr','pad','phone','exoskeleton'])
def test_same_gripper_different_inputs(source):
    engine=BindingEngine(runtime('new.gripper','m',[.05,0],source))
    assert engine.targets(InputFrame(source,'epoch',0,100,'input',{'scalar':1.}))[0]['values']==[0.]


def test_one_input_mixed_dual_arms():
    joints=[{'name':'j1','unit':'rad'},{'name':'j2','unit':'rad'}]
    bindings=[{'id':key,'source':'pad.axes','target':key+'.position_target',
               'controller':'morla.joint_jog','parameters':{'axes_map':['x','y'],'speed':1}}
              for key in ('left','right')]
    rt=SimpleNamespace(clock=lambda:100,instances={key:{'plugin_id':plugin,'joints':joints}
        for key,plugin in [('left','vendor.new_arm'),('right','another.robot')]},
        adapters={key:SimpleNamespace(health=lambda now:{'values':[.1,.2]}) for key in ('left','right')},
        resolved={'bindings':bindings})
    clock=[100]
    rt.clock=lambda:clock[0]
    engine=BindingEngine(rt)
    targets=engine.targets(InputFrame('pad','epoch',0,100,'input',{'axes':{'x':.5,'y':-.5}}))
    assert {row['component_id'] for row in targets}=={'left','right'}
    assert all(row['values']==[.1,.2] for row in targets)

    clock[0]+=20_000_000
    moved=engine.targets(InputFrame('pad','epoch',1,clock[0],'input',{'axes':{'x':.5,'y':-.5}}))
    assert all(row['values']==pytest.approx([.11,.19]) for row in moved)


def test_headless_host_dispatches_shared_engine_without_web():
    from morla_teleop.session import ControlHost
    rt=runtime('unknown.new_gripper','m',[.05,0])
    rt.status='ACTIVE';rt.session_id='headless'
    rows=[]
    rt.submit=lambda row:rows.append(row) or {'accepted':True}
    host=ControlHost(rt)
    assert host.ingest(InputFrame('input','epoch',0,100,'input',{'scalar':1.}))['accepted']
    assert rows[0]['commands'][0]['values']==[0.]
    host.close()


def test_same_pinch_mapping_for_ratio_and_metric_grippers():
    for plugin,unit,output,expected in [('astral','ratio',[0,1],1),('epg','m',[.05,0],0)]:
        rt=runtime(plugin,unit,output)
        binding=rt.resolved['bindings'][0]
        binding.update(source='input.landmarks',controller='morla.pinch_scalar',parameters={'output_range':output})
        points=[[0.,0.,0.] for _ in range(21)]
        points[8]=[.01,0,0]
        target=BindingEngine(rt).targets(InputFrame('input','epoch',0,100,'hand',{'landmarks':points}))
        assert target[0]['values']==pytest.approx([expected])


@pytest.mark.parametrize('source',['vr','vader','mobile'])
@pytest.mark.parametrize('plugin,unit,output,expected',[
    ('morla.jodell.epg50_060','m',[.05,0],.025),
    ('morla.astral.gripper','ratio',[0,1],.5)])
def test_input_actuator_configuration_matrix(source,plugin,unit,output,expected):
    rt=runtime(plugin,unit,output,source)
    target=BindingEngine(rt).targets(InputFrame(source,'epoch',0,100,'input',{'scalar':.5}))
    assert target[0]['values']==pytest.approx([expected])


def test_unknown_controller_fails_instead_of_silently_skipping():
    rt=runtime('vendor.arm','rad',[0,1])
    rt.resolved['bindings'][0]['controller']='uninstalled.controller'
    with pytest.raises(Exception,match='missing|ambiguous|unavailable'):
        BindingEngine(rt).targets(InputFrame('input','epoch',0,100,'input',{'scalar':.5}))


def test_controller_cannot_return_wrong_target_dimension():
    rt=runtime('unknown.gripper','m',[0,1])
    engine=BindingEngine(rt)
    controller=engine._binding_controller(rt.resolved['bindings'][0])
    controller.registry.register('bad.shape',lambda *args:[0.,1.])
    controller.binding['controller']='bad.shape'
    with pytest.raises(ValueError,match='dimension'):
        engine.targets(InputFrame('input','epoch',0,100,'input',{'scalar':.5}))


def test_cached_scalar_is_not_refiltered_by_fresh_unrelated_pose():
    rt=runtime('generic.gripper','ratio',[0,1]);engine=BindingEngine(rt)
    frame=InputFrame('input','epoch',0,100,'input',{'scalar':.5},
        port_metadata={'scalar':{'received_ns':100,'frame_id':'trigger'}})
    assert len(engine.targets(frame))==1
    # Same scalar sample, new transport event: no second controller/filter update.
    repeated=InputFrame('input','epoch',1,100,'input',{'scalar':.5,'pose':[0]*7},
        port_metadata=frame.port_metadata)
    assert engine.targets(repeated)==[]
    engine.reset()
    assert len(engine.targets(repeated))==1


def test_headless_close_still_releases_controllers_after_input_close_failure():
    from morla_teleop.session import ControlHost
    rt=runtime('generic.gripper','m',[.05,0]);host=ControlHost(rt)
    host.inputs=SimpleNamespace(close=lambda:(_ for _ in ()).throw(RuntimeError('reader close failed')))
    closed=[];host.engine.close=lambda:closed.append(True) or []
    host.close()
    assert closed==[True] and host.inputs is None and host.errors==['reader close failed']


def test_operator_actions_use_same_dispatch_in_headless_and_web():
    from morla_teleop.actions import dispatch_input_action
    calls=[]
    op=lambda name,request:calls.append((name,request))
    for action in ('teleop_start','teleop_pause','estop'):
        dispatch_input_action(action,'request',operation=op)
    assert calls==[('activate','request'),('disarm','request'),('estop','request')]
    with pytest.raises(ValueError,match='handler'):
        dispatch_input_action('record_start','request',operation=op)


def test_cartesian_controller_uses_installed_kinematics_without_brand_branch(monkeypatch):
    from types import SimpleNamespace
    import numpy as np
    from morla_teleop.kinematics import solver_for
    provider=SimpleNamespace(name='unknown.robot',load=lambda:lambda instance:SimpleNamespace(
        sync_state=lambda q:None,fk=lambda q:np.eye(4),solve=lambda target:[0.]*3))
    monkeypatch.setattr('importlib.metadata.entry_points',lambda **kwargs:[provider])
    solver=solver_for({'plugin_id':'unknown.robot','parameters':{}})
    assert solver.solve(np.eye(4))==[0.,0.,0.]


def test_partial_input_frame_does_not_block_other_bound_devices():
    rt=runtime('generic.gripper','ratio',[0,1])
    rt.instances['second']=rt.instances['gripper'].copy()
    rt.adapters['second']=rt.adapters['gripper']
    rt.resolved['bindings'].append({'id':'second','source':'input.right_scalar',
        'target':'second.position_target','controller':'morla.scalar_map','parameters':{}})
    engine=BindingEngine(rt)
    targets=engine.targets(InputFrame('input','epoch',0,100,'input',{'right_scalar':.7}))
    assert len(targets)==1 and targets[0]['component_id']=='second'
    assert targets[0]['values']==[.7]
