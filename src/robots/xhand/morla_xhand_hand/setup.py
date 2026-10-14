from glob import glob
from pathlib import Path
from setuptools import setup

package_name = "morla_xhand_hand"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        *[(f"share/{package_name}/{path.rsplit('/', 1)[0]}", [path]) for path in glob("config/**/*", recursive=True) if Path(path).is_file()],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MORLA maintainers",
    maintainer_email="robot@localhost",
    description="XHand 12-joint hand component package",
    license="Apache-2.0",
    entry_points={"console_scripts": []},
)
