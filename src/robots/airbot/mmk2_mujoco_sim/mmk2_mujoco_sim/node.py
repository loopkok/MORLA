def main():
    import sys
    from morla_core.resource_locator import resolve_plugin_resource
    from morla_web_manager.session_cli import main as run
    args = sys.argv[1:]
    if '--assembly' not in args:
        args = ['--assembly', str(resolve_plugin_resource('mmk2_mujoco_sim', 'config/composable.sim.yaml')), *args]
    return run(args)
