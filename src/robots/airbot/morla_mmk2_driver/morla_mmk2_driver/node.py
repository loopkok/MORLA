"""ROS 2 entry point through the public MORLA gateway."""
def main(args=None):
    from morla_runtime.ros_bridge import main as run
    return run(args,node_name='mmk2_driver',required_plugin=None,publish_joint_states=True)
