"""Robot-owned entry point using the shared session host and simulation."""
def main():
    import sys
    from morla_core.resource_locator import package_share
    from morla_web_manager.session_cli import main as run
    args = sys.argv[1:]
    if '--assembly' not in args:
        args = ['--assembly', str(package_share('fr3_mujoco_sim')/'config/composable.sim.yaml'), *args]
    return run(args)
