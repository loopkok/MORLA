from glob import glob
import os

from setuptools import find_packages, setup

package_name = "astral_arm_teleop"


def _existing(paths):
    return [p for p in paths if os.path.isfile(p)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "astral_arm_teleop": [
            "robot/urdf/*.urdf",
            "robot/RobotMain_URDF/meshes/*",
        ],
    },
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), _existing(glob("config/*"))),
        (os.path.join("share", package_name, "launch"), _existing(glob("launch/*.launch.py"))),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=False,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Astral dual-arm teleop (DH + URDF IK) — arm-only",
    license="MIT",
    entry_points={
        "console_scripts": [
            "astral_arm_teleop_node = astral_arm_teleop.astral_arm_teleop_node:main",
            "ik_solver_node = astral_arm_teleop.ik_solver_node:main",
            "keyboard_vr_sim = astral_arm_teleop.keyboard_vr_sim:main",
            "body_joints_sim = astral_arm_teleop.body_joints_sim:main",
            "test_ik_solver = astral_arm_teleop.test_ik_solver:main",
            "test_geometric_ik = astral_arm_teleop.test_geometric_ik:main",
            "test_dh_urdf_fk = astral_arm_teleop.test_dh_urdf_fk:main",
            "fit_dh_from_urdf = astral_arm_teleop.fit_dh_from_urdf:main",
            "test_vr_mapping = astral_arm_teleop.test_vr_mapping:main",
            "test_dataflow = astral_arm_teleop.test_dataflow:main",
            "test_safe_teleop = astral_arm_teleop.test_safe_teleop:main",
            "teleop_tune_plot = astral_arm_teleop.teleop_tune_plot:main",
        ],
    },
)
