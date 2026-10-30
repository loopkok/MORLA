from glob import glob
from setuptools import find_packages,setup
setup(name='morla_vader5_input',version='0.1.0',packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages',['resource/morla_vader5_input']),
        ('share/ament_index/resource_index/morla_components',['resource/morla_vader5_input_components']),
        ('share/morla_vader5_input',['package.xml','descriptor.json','README.md','LICENSE','PROVENANCE.json']),
        ('share/morla_vader5_input/config',glob('config/*.yaml')),
        ('share/morla_vader5_input/launch',glob('launch/*.launch.py'))],
    install_requires=['setuptools'],license='GPL-3.0-or-later',
    maintainer='MORLA maintainers',maintainer_email='robot@localhost',
    description='Flydigi Vader 5 Pro first-class ROS/MORLA input, reusing the supplied SDK',
    entry_points={'morla.processes':['morla.input.vader5 = morla_vader5_input.processes:plan'],'console_scripts':['vader5_input = morla_vader5_input.node:main'],
        'morla.ros_inputs':['morla.input.vader5 = morla_vader5_input.source:create_ros_source']})
