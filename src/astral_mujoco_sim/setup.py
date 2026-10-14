from glob import glob
import os

from setuptools import find_packages, setup

package_name = "astral_mujoco_sim"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "config"), glob("config/*")),
    (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    (os.path.join("share", package_name, "assets", "mjcf"), glob("assets/mjcf/*")),
]
mesh_glob = glob("assets/meshes/*")
if mesh_glob:
    data_files.append(
        (os.path.join("share", package_name, "assets", "meshes"), mesh_glob)
    )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Astral dual-arm MuJoCo sim (joint_commands → MJCF)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "astral_mujoco_sim_node = astral_mujoco_sim.mujoco_sim_node:main",
        ],
    },
)
