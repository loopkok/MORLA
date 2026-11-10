from setuptools import setup, find_packages
from glob import glob
setup(name='mmk2_mujoco_sim', version='0.1.0', packages=find_packages(exclude=['test']),
 data_files=[('share/ament_index/resource_index/packages',['resource/mmk2_mujoco_sim']),
 ('share/ament_index/resource_index/morla_components',['resource/mmk2_mujoco_sim_components']),
 ('share/mmk2_mujoco_sim',['package.xml','descriptors.json','README.md']),
 ('share/mmk2_mujoco_sim/config',glob('config/*.yaml')),
 ('share/mmk2_mujoco_sim/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools','numpy','scipy','mujoco>=3.2,<4','Pillow'],
 license='Apache-2.0',maintainer='MORLA',maintainer_email='maintainers@morla.dev',
 entry_points={'morla.controllers':['morla.mmk2.differential_drive = mmk2_mujoco_sim.controllers:create_controller'],
 'morla.simulation_scenes':['morla.airbot.mmk2 = mmk2_mujoco_sim.scene:create_adapters'],
 'morla.kinematics':['morla.airbot.mmk2.left_arm = mmk2_mujoco_sim.kinematics:create_kinematics',
 'morla.airbot.mmk2.right_arm = mmk2_mujoco_sim.kinematics:create_kinematics'],
 'console_scripts':['mmk2_mujoco_sim_node = mmk2_mujoco_sim.node:main']})
