from glob import glob
import os

from setuptools import find_packages, setup

package_name = "astral_policy_inference"


def _existing(paths):
    return [p for p in paths if os.path.isfile(p)]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), _existing(glob("config/*"))),
        (os.path.join("share", package_name, "launch"), _existing(glob("launch/*.launch.py"))),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=False,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="Astral VLA policy inference/replay/HITL: openpi pi0.5 (websocket) + lerobot ACT (in-process) backends, sync/async/RTC engines, real-robot replay, VR takeover",
    license="MIT",
    entry_points={
        "console_scripts": [
            "policy_node = astral_policy_inference.node:main",
            "policy_keyboard = astral_policy_inference.keyboard:main",
        ],
    },
)
