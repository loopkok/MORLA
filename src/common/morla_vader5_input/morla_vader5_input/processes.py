"""Vader acquisition only; robot selection belongs to assembly bindings."""
def plan(instance):
    p=instance.get('parameters',{})
    argv=['ros2','run','morla_vader5_input','vader5_input','--ros-args',
          '-p','device:='+str(p.get('device','')),
          '-p','fake:='+str(instance.get('backend')=='fake').lower(),
          '-p','topic:='+p.get('input_topic','/vader5/events')]
    # The SDK reads at ~490 Hz; publish_hz only caps the ROS event rate (1..100).
    if 'publish_hz' in p:argv+=['-p','publish_hz:='+str(p['publish_hz'])]
    return [argv]
