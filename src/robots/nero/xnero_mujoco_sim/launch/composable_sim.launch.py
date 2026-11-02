"""Public backend entry; legacy launches remain available."""
from morla_runtime.sim_launch import description

def generate_launch_description():
    return description('xnero_mujoco_sim')
