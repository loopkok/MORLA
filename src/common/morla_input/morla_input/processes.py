"""Device-local process plans. No robot targets or combination launch logic."""
def quest(instance):
    p=instance.get('parameters',{})
    if instance.get('backend')=='fake': return []
    return [['ros2','run','morla_input','quest3_udp_mocap','--ros-args',
             '-p','publish_teleop_events:=true','-p','controller_as_wrist:=true','-p','viz:=false',
             '-p','protocol:='+p.get('protocol','tcp_wired'),
             '-p','tcp_port:='+str(p.get('tcp_port',8000)),
             '-p','udp_port:='+str(p.get('udp_port',9000)),
             '-r','quest3/right_teleop_events:='+p.get('input_topic','/quest3/right_teleop_events')]]


def gamepad(instance):
    p=instance.get('parameters',{})
    if instance.get('backend')=='fake':return []
    return [['ros2','run','joy','game_controller_node','--ros-args',
             '-r','joy:='+p.get('input_topic','/joy'),
             '-p','device_name:='+p.get('device_name',''),
             '-p','deadzone:='+str(p.get('deadzone',.08))]]
