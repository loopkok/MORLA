from setuptools import find_packages, setup

setup(
    name='morla_mmk2_teleop', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/morla_mmk2_teleop']),
        ('share/ament_index/resource_index/morla_components', ['resource/morla_mmk2_teleop_components']),
        ('share/morla_mmk2_teleop', ['package.xml', 'controller.json', 'README.md', 'LICENSE']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    license='Apache-2.0', maintainer='MORLA', maintainer_email='maintainers@morla.dev',
    entry_points={'morla.controllers': [
        'morla.mmk2.cartesian_delta = morla_mmk2_teleop.vr_controller:create_controller',
    ]},
)
