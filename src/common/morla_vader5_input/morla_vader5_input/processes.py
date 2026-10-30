"""Vader acquisition only; robot selection belongs to assembly bindings."""
def plan(instance):
    p=instance.get('parameters',{})
    return [['ros2','run','morla_vader5_input','vader5_input','--ros-args',
             '-p','device:='+str(p.get('device','')),
             '-p','fake:='+str(instance.get('backend')=='fake').lower(),
             '-p','topic:='+p.get('input_topic','/vader5/events')]]
