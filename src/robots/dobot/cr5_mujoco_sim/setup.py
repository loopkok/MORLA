from setuptools import setup, find_packages
from glob import glob
setup(name='cr5_mujoco_sim', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/cr5_mujoco_sim']),
                  ('share/cr5_mujoco_sim',['package.xml','README.md']),
                  ('share/cr5_mujoco_sim/config',glob('config/*.yaml')),
                  ('share/cr5_mujoco_sim/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'], license='Apache-2.0',
      maintainer='MORLA', maintainer_email='robot@localhost',
      entry_points={'console_scripts':['cr5_mujoco_sim_node = cr5_mujoco_sim.node:main']})
