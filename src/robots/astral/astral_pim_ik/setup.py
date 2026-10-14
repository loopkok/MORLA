from glob import glob
import os

from setuptools import find_packages, setup

package_name = "astral_pim_ik"


def _existing(paths):
    return [p for p in paths if os.path.isfile(p)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), _existing(glob("config/*"))),
    ],
    # torch / pinocchio are runtime-optional: geometry.py needs pinocchio (via
    # astral_arm_teleop.ik.geometric), network/kinematics/train need torch.
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=False,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="PiM-IK two-stage IK for the Astral arm (NN arm angle + DH-free geometric IK)",
    license="MIT",
)
