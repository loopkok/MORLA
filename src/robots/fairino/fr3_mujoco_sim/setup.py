from setuptools import setup, find_packages
from glob import glob
setup(name='fr3_mujoco_sim', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/fr3_mujoco_sim']),
                  ('share/fr3_mujoco_sim',['package.xml','README.md']),
                  ('share/fr3_mujoco_sim/config',glob('config/*.yaml')),
                  ('share/fr3_mujoco_sim/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'], license='Apache-2.0',
      maintainer='MORLA', maintainer_email='maintainers@morla.dev',
      entry_points={'console_scripts':['fr3_mujoco_sim_node = fr3_mujoco_sim.node:main']})
