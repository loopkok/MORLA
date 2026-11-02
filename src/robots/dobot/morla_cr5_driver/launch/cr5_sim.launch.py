"""Compatibility entry for the public CR5 simulation backend."""
from morla_runtime.sim_launch import description

def generate_launch_description():
    return description('morla_cr5_driver', 'quest.sim.yaml')
