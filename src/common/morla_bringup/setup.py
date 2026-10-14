from glob import glob
import os

from setuptools import find_packages, setup

package_name = "morla_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        *[(os.path.join("share", package_name, os.path.dirname(path)), [path])
          for path in glob("config/**/*", recursive=True) if os.path.isfile(path)],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Whole-robot teleop bringup: compose existing skill/driver launches",
    license="MIT",
    entry_points={
        "console_scripts": [
            "controller_start_gate = morla_bringup.controller_start_gate:main",
            "controller_workpos_gate = morla_bringup.controller_workpos_gate:main",
            "head_teleop_node = morla_bringup.head_teleop_node:main",
            "control_coordinator_node = morla_bringup.control_coordinator_node:main",
        ]
    },
)
