from glob import glob
import os

from setuptools import find_packages, setup

package_name = "astral_robot_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Astral RobotMain ROS2 driver (astral_robot_sdk)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "astral_robot_driver = astral_robot_control.driver_node:main",
        ],
    },
)
