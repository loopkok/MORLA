from glob import glob
import os

from setuptools import find_packages, setup

package_name = "wujihand_mujoco_sim"

# Install MJCF + meshes under share/
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "config"), glob("config/*")),
    (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    (os.path.join("share", package_name, "assets", "mjcf"), glob("assets/mjcf/*")),
]
for side in ("left", "right"):
    mesh_glob = glob(f"assets/meshes/{side}/*")
    if mesh_glob:
        data_files.append(
            (os.path.join("share", package_name, "assets", "meshes", side), mesh_glob)
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
    description="Wuji Hand MuJoCo sim + tuning viewer (joint_commands / landmarks)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wujihand_mujoco_sim_node = wujihand_mujoco_sim.mujoco_sim_node:main",
            "wujihand_tuning_node = wujihand_mujoco_sim.tuning_viewer_node:main",
        ],
    },
)
