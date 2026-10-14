import os
from glob import glob
from setuptools import find_packages, setup

package_name = "xhand_retargeting"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yml") + glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="XHand retargeting from Quest3 VR hand mocap data",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "xhand_dex_retargeting_node = xhand_retargeting.xhand_dex_retargeting_node:main",
        ],
    },
)
