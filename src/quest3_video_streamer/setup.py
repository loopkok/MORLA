from setuptools import find_packages, setup

package_name = 'quest3_video_streamer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/realsense.launch.py',
            'launch/usb_camera.launch.py',
            'launch/multi_camera.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loopkok',
    maintainer_email='loopkok@todo.todo',
    description='Push USB camera and RealSense D435i video to Quest 3 over WebRTC.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'quest3_video_streamer = quest3_video_streamer.streamer_node:main',
        ],
    },
)
