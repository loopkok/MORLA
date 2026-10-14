from glob import glob
import os

from setuptools import find_packages, setup

package_name = "xnero_mujoco_sim"

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
    description="xnero MuJoCo sim (hand physics + kinematic arms, tactile on /xhand_state)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "xnero_mujoco_sim_node = xnero_mujoco_sim.sim_node:main",
        ],
    },
)
