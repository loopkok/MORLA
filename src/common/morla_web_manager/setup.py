from glob import glob
import os

from setuptools import find_packages, setup

package_name = "morla_web_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web", "dist"),
         [path for path in glob("web/dist/*.*") if os.path.isfile(path)]),
        (os.path.join("share", package_name, "web", "dist", "assets"),
         [path for path in glob("web/dist/assets/*") if os.path.isfile(path)]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="loopkok",
    maintainer_email="loopkok@todo.todo",
    description="MORLA web manager for robot operation and monitoring",
    license="MIT",
    entry_points={
        "console_scripts": [
            "morla_web_manager = morla_web_manager.web_server:main",
        ],
    },
)
