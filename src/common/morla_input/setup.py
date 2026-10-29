from setuptools import find_packages, setup

package_name = 'morla_input'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/morla_components', ['resource/morla_input_components']),
        ('share/' + package_name, ['quest_right_descriptor.json']),
        ('share/' + package_name + '/config', ['config/quest3_mocap.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loopkok',
    maintainer_email='loopkok@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
            'console_scripts': [
                'quest3_udp_mocap = morla_input.quest3_udp_mocap:main',
            ],
        },
)
