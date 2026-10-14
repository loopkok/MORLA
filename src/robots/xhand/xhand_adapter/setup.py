from setuptools import find_packages, setup

package_name = "xhand_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="MORLA maintainers",
    maintainer_email="robot@localhost",
    description="Strict standard JointState to native XHand ROS adapter",
    license="Apache-2.0",
    entry_points={"console_scripts": ["xhand_adapter = xhand_adapter.adapter_node:main"]},
)
