from glob import glob
from setuptools import find_packages, setup

setup(
    name='morla_fr3_driver', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/morla_fr3_driver']),
        ('share/ament_index/resource_index/morla_components', ['resource/morla_fr3_driver_components']),
        ('share/morla_fr3_driver', ['package.xml', 'descriptor.json', 'README.md',
                                  'PROVENANCE.json', 'LICENSE', 'LICENSE-FAIRINO']),
        ('share/morla_fr3_driver/config', glob('config/*.yaml')),
        ('share/morla_fr3_driver/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    extras_require={'real': ['python-mbedtls>=2.9,<3']},
    license='Apache-2.0', maintainer='MORLA maintainers', maintainer_email='robot@localhost',
    description='FAIRINO FR3 SDK-backed ROS 2 and composable MORLA driver',
    entry_points={
        'morla.adapters': [
            'morla.fairino.fr3:real = morla_fr3_driver.adapter:create_adapter',
            'morla.fairino.fr3:fake = morla_fr3_driver.adapter:create_adapter',
        ],
        'morla.kinematics': ['morla.fairino.fr3 = morla_fr3_driver.kinematics:create_kinematics'],
        'console_scripts': ['fr3_driver = morla_fr3_driver.node:main',
                            'fr3_check = morla_fr3_driver.check:main'],
    },
)
