from glob import glob
from setuptools import find_packages, setup

setup(
    name='morla_loophand_driver', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/morla_loophand_driver']),
        ('share/ament_index/resource_index/morla_components', ['resource/morla_loophand_driver_components']),
        ('share/morla_loophand_driver', ['package.xml', 'descriptor.json', 'README.md', 'PROVENANCE.json', 'LICENSE']),
        ('share/morla_loophand_driver/config', glob('config/*.yaml')),
        ('share/morla_loophand_driver/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'pyserial'],
    license='Apache-2.0',
    maintainer='MORLA maintainers', maintainer_email='robot@localhost',
    description='Independent loophand_driver ROS 2 and MORLA component',
    entry_points={
        'morla.adapters': [
            'morla.loophand.compact:real = morla_loophand_driver.adapter:create_adapter',
            'morla.loophand.compact:fake = morla_loophand_driver.adapter:create_adapter',
        ],
        'console_scripts': ['loophand_driver = morla_loophand_driver.node:main'],
    },
)
