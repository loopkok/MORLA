from glob import glob
import os

from setuptools import find_packages, setup

package_name = "astral_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Whole-robot teleop bringup: compose existing skill/driver launches",
    license="MIT",
    entry_points={
        "console_scripts": [
            "controller_start_gate = astral_teleop.controller_start_gate:main",
            "controller_workpos_gate = astral_teleop.controller_workpos_gate:main",
            "head_teleop_node = astral_teleop.head_teleop_node:main",
        ]
    },
)
