"""Compatibility entry for the public FR3 simulation backend."""
from morla_runtime.sim_launch import description

def generate_launch_description():
    return description('morla_fr3_driver', 'quest.sim.yaml')
