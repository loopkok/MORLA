from glob import glob
from pathlib import Path
from setuptools import setup, find_packages

package_name = "morla_core"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    package_data={"morla_core.plugins": ["data/*.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        *[(f"share/{package_name}/{Path(path).parent}", [path])
          for path in glob("config/**/*", recursive=True) + glob("schemas/*", recursive=True) if Path(path).is_file()],
    ],
    install_requires=["setuptools", "numpy", "PyYAML>=6", "jsonschema>=4"],
    zip_safe=False,
    maintainer="MORLA maintainers",
    maintainer_email="robot@localhost",
    description="ROS-free shared schemas, profiles and ownership primitives",
    license="Apache-2.0",
    entry_points={"console_scripts": ["morla-profile = morla_core.profile:main", "morla-system = morla_core.composition.cli:main"]},
)
