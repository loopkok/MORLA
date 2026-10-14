from glob import glob
import os

from setuptools import find_packages, setup

package_name = "astral_gripper_teleop"

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
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Quest3 pinch → gripper command (device-swappable adapter)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pinch_gripper_node = astral_gripper_teleop.pinch_gripper_node:main",
        ],
    },
)
