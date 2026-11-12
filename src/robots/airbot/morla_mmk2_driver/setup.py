from glob import glob
from setuptools import setup, find_packages
from morla_mmk2_driver.contract import PREFIX, ROLES
setup(name='morla_mmk2_driver',version='0.1.0',packages=find_packages(exclude=['test']),
 package_data={'morla_mmk2_driver.vendor':['proto/*.proto']},
 data_files=[('share/ament_index/resource_index/packages',['resource/morla_mmk2_driver']),
 ('share/ament_index/resource_index/morla_components',['resource/morla_mmk2_driver_components']),
 ('share/morla_mmk2_driver',['package.xml','descriptors.json','README.md','PROVENANCE.json','LICENSE','LICENSE-SDK']),
 ('share/morla_mmk2_driver/config',glob('config/*.yaml')),('share/morla_mmk2_driver/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools','grpcio>=1.73.1','protobuf>=6.31,<7','numpy','Pillow'],
 extras_require={'ik':['scipy','mujoco>=3.2,<4']},license='Apache-2.0',maintainer='MORLA',maintainer_email='maintainers@morla.dev',
 entry_points={'morla.adapters':[PREFIX+role+':'+backend+' = morla_mmk2_driver.adapter:create_adapter' for role in [*ROLES,'camera'] for backend in ['fake','real']],
 'morla.kinematics':[PREFIX+side+'_arm = morla_mmk2_driver.kinematics:create_kinematics' for side in ['left','right']],
 'morla.controllers':['morla.mmk2_sdk.base_twist = morla_mmk2_driver.controllers:create_controller',
                      'morla.mmk2_sdk.lift_button_jog = morla_mmk2_driver.controllers:create_lift_jog'],
 'console_scripts':['mmk2_driver = morla_mmk2_driver.node:main','mmk2_check = morla_mmk2_driver.check:main']})
