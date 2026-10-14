from glob import glob
import os

from setuptools import find_packages, setup

package_name = "wuji_glove"

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
    description="Wuji Glove → hand_landmarks PoseArray (quest3-compatible)",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "wuji_glove_mocap = wuji_glove.wuji_glove_mocap:main",
        ],
    },
)
