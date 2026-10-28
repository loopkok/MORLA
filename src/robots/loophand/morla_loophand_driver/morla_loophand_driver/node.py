"""Single-device ROS entry point backed by the common runtime gateway."""
def main(args=None):
    from morla_runtime.ros_bridge import main as run
    return run(args, node_name='loophand_driver', required_plugin='morla.loophand.compact',
               publish_joint_states=True)
