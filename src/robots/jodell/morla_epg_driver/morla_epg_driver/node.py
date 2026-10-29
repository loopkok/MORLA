"""ROS 2 ingress remains behind MORLA's runtime command/authority gateway."""

def main(args=None):
    from morla_runtime.ros_bridge import main as run
    return run(args,node_name='epg_driver',required_plugin='morla.jodell.epg50_060',publish_joint_states=True)
