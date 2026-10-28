from glob import glob
from setuptools import find_packages, setup

setup(
    name='morla_cr5_driver', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/morla_cr5_driver']),
        ('share/ament_index/resource_index/morla_components', ['resource/morla_cr5_driver_components']),
        ('share/morla_cr5_driver', ['package.xml', 'descriptor.json', 'README.md', 'PROVENANCE.json', 'LICENSE', 'LICENSE-DOBOT']),
        ('share/morla_cr5_driver/config', glob('config/*.yaml')),
        ('share/morla_cr5_driver/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    license='Apache-2.0; MIT (vendored Dobot layout)',
    maintainer='MORLA maintainers', maintainer_email='robot@localhost',
    description='Independent cr5_driver ROS 2 and MORLA component',
    entry_points={
        'morla.adapters': [
            'morla.dobot.cr5:real = morla_cr5_driver.adapter:create_adapter',
            'morla.dobot.cr5:fake = morla_cr5_driver.adapter:create_adapter',
        ],
        'console_scripts': ['cr5_driver = morla_cr5_driver.node:main'],
    },
)
