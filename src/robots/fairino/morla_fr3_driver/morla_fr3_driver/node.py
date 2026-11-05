"""Single-device entry point; commands use the common MORLA gateway."""


def main(args=None):
    from morla_runtime.ros_bridge import main as run
    return run(args, node_name='fr3_driver', required_plugin='morla.fairino.fr3',
               publish_joint_states=True)
