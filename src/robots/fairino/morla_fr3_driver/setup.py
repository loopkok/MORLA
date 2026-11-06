from glob import glob
from setuptools import find_packages, setup

setup(
    name='morla_fr3_driver', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/morla_fr3_driver']),
        ('share/ament_index/resource_index/morla_components', ['resource/morla_fr3_driver_components']),
        ('share/morla_fr3_driver', ['package.xml', 'descriptor.json', 'controller.json', 'jog_controller.json', 'README.md',
                                  'PROVENANCE.json', 'LICENSE', 'LICENSE-FAIRINO']),
        ('share/morla_fr3_driver/config', glob('config/*.yaml')),
        ('share/morla_fr3_driver/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    extras_require={'sim':['mujoco>=3.2,<4','Pillow'], 'ik':['pin>=2.6,<5'], 'real': []},
    license='Apache-2.0', maintainer='MORLA maintainers', maintainer_email='maintainers@morla.dev',
    description='FAIRINO FR3 SDK V2.1.5 / Robot V3.8.5 ROS 2 and MORLA driver',
    entry_points={
        'morla.adapters': [
            'morla.fairino.fr3:real = morla_fr3_driver.adapter:create_adapter',
            'morla.fairino.fr3:fake = morla_fr3_driver.adapter:create_adapter',
        ],
        'morla.kinematics': ['morla.fairino.fr3 = morla_fr3_driver.kinematics:create_kinematics'],
        'morla.controllers': ['morla.fr3.cartesian_delta = morla_fr3_driver.vr_controller:create_controller',
                              'morla.fr3.cartesian_jog = morla_fr3_driver.vr_controller:create_jog_controller'],
        'console_scripts': ['fr3_driver = morla_fr3_driver.node:main',
                            'fr3_check = morla_fr3_driver.check:main'],
    },
)
