from setuptools import find_packages, setup
setup(name="morla_teleop", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/morla_teleop"]),
                  ("share/morla_teleop", ["package.xml"])], install_requires=["setuptools", "numpy"])
