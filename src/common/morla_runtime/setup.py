from setuptools import setup, find_packages
from glob import glob
setup(name='morla_runtime', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/morla_runtime']),
                  ('share/morla_runtime', ['package.xml', 'README.md']),
                  ('share/morla_runtime/config', glob('config/*')),
                  ('share/morla_runtime/launch', glob('launch/*'))],
      install_requires=['setuptools'], entry_points={'console_scripts': [
          'morla-runtime = morla_runtime.cli:main', 'runtime_node = morla_runtime.ros_bridge:main',
          'joint_state_runtime_node = morla_runtime.ros_bridge:joint_state_main']})
