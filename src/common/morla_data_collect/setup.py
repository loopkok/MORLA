from glob import glob
import os

from setuptools import find_packages, setup

package_name = "morla_data_collect"


def _existing(paths):
    return [p for p in paths if os.path.isfile(p)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    include_package_data=True,
    package_data={"morla_data_collect.pipeline": ["*.sh", "README.md"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), _existing(glob("config/*"))),
        (os.path.join("share", package_name, "launch"), _existing(glob("launch/*.launch.py"))),
    ],
    install_requires=["setuptools", "numpy", "h5py"],
    zip_safe=False,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Astral VLA data collection, alignment, validation, LeRobot v2.1 export, Rerun replay",
    license="MIT",
    entry_points={
        "console_scripts": [
            "data_collect_node = morla_data_collect.data_collect_node:main",
            "keyboard_controller = morla_data_collect.keyboard_controller:main",
            "vr_collect_control = morla_data_collect.vr_collect_control:main",
            "data_pipeline = morla_data_collect.pipeline.cli:main",
        ],
    },
)
