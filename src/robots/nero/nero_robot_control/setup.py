from glob import glob
import os
from pathlib import Path
from setuptools import find_packages, setup

package_name = "nero_robot_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *[
            (os.path.join("share", package_name, os.path.dirname(path)), [path])
            for path in glob("config/**/*", recursive=True)
            if Path(path).is_file()
        ],
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="MORLA maintainers",
    maintainer_email="robot@localhost",
    description="Single-writer Nero arm driver with strict standard JointState interfaces",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "nero_robot_driver = nero_robot_control.driver_node:main",
        ],
    },
)
