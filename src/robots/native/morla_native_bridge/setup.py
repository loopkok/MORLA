from setuptools import setup,find_packages
setup(name='morla_native_bridge',version='0.1.0',packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages',['resource/morla_native_bridge']),
                ('share/morla_native_bridge',['package.xml','README.md'])],
    install_requires=['setuptools'],entry_points={'morla.kinematics':['morla.nero.arm = morla_native_bridge.kinematics:nero','morla.astral.arm = morla_native_bridge.kinematics:astral'], 'morla.adapters':[
        plugin+':real = morla_native_bridge.factory:create_adapter'
        for plugin in ['morla.nero.arm','morla.astral.arm','morla.astral.gripper','morla.xhand.hand','morla.wuji.hand']]})
