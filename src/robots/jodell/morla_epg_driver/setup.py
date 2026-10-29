from glob import glob
from setuptools import find_packages,setup
setup(name='morla_epg_driver',version='0.1.0',packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/morla_epg_driver']),
        ('share/ament_index/resource_index/morla_components',['resource/morla_epg_driver_components']),
        ('share/morla_epg_driver',['package.xml','descriptor.json','README.md','PROVENANCE.json','LICENSE']),
        ('share/morla_epg_driver/config',glob('config/*.yaml')),
        ('share/morla_epg_driver/launch',glob('launch/*.launch.py')),
    ],install_requires=['setuptools','pyserial','modbus-tk'],
    license='Apache-2.0 (MORLA wrapper only; vendor SDK installed separately)',
    maintainer='MORLA maintainers',maintainer_email='robot@localhost',
    description='Independent Jodell EPG50-060 RS485 gripper ROS2/MORLA driver',
    entry_points={'morla.adapters':[
        'morla.jodell.epg50_060:real = morla_epg_driver.adapter:create_adapter',
        'morla.jodell.epg50_060:fake = morla_epg_driver.adapter:create_adapter'],
        'console_scripts':['epg_driver = morla_epg_driver.node:main','epg_probe = morla_epg_driver.probe:main']})
