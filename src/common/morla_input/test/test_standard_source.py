import json
from morla_input.standard_source import StandardRosSource


def test_same_protocol_for_arbitrary_upstream():
    for name in ('vr','pad','phone','exo'):
        bridge=StandardRosSource({'instance_id':name,'namespace':'/cell/'+name},clock=lambda:100,on_transition=lambda:None)
        frame=bridge.consume(json.dumps({'epoch':'new','sequence':0,'ports':{'scalar':.5}}))
        assert frame.source_id==name and frame.ports=={'scalar':.5}
        assert bridge.topic=='/cell/'+name+'/intent'
        bridge.consume(json.dumps({'connected':False}))
        assert not bridge.source.connected


def test_named_axes_and_scalar_keep_individual_sample_times():
    from morla_input.adapters.ingress import InputIngressAdapter
    from morla_input.adapters.normalized import InputFrame
    adapter=InputIngressAdapter({'instance_id':'vr','ports':{
        'left_axes':{'semantic':'axes_buttons','axis_names':['x','y']}}},lambda:100)
    adapter.configure({})
    adapter.feed(InputFrame('vr','epoch',0,100,'input',{'left_axes':{'x':.2,'y':.4},'left_scalar':.5},
        port_metadata={'left_scalar':{'received_ns':50},'left_axes':{'received_ns':70}}))
    ports=adapter.read(100)['ports']
    assert ports['left_axes']['values']==[.2,.4]
    assert ports['left_scalar']['received_ns']==50


def test_quest_joy_carries_only_the_thumbstick():
    from types import SimpleNamespace
    from morla_input.ros_sources import RosInputManager
    from morla_input.quest_events import QuestEventSource
    frames=[];clock=[100]
    manager=RosInputManager([],frames.append,lambda source:None,clock=lambda:clock[0])
    source=QuestEventSource('vr',clock=lambda:clock[0],sides=('left','right'));source.reconnect()
    manager.sources['vr']=source;manager.cache['vr']={}
    manager._quest_joy('vr','left',SimpleNamespace(axes=[.2,.8,0,.5],header=SimpleNamespace(frame_id='left')))
    clock[0]=200
    manager._quest_joy('vr','right',SimpleNamespace(axes=[.8,.0,.1,.2],header=SimpleNamespace(frame_id='right')))
    assert frames[-1].ports['left_axes']=={'x':0.,'y':.5}
    assert frames[-1].ports['right_axes']=={'x':.1,'y':.2}
    # Grip, trigger and landmarks own ports in the event stream.
    assert 'left_grip' not in frames[-1].ports and 'scalar' not in frames[-1].ports
