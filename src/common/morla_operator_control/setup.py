from glob import glob
from setuptools import find_packages, setup

setup(
    name="morla_operator_control", version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/morla_operator_control"]),
        ("share/morla_operator_control", ["package.xml"]),
        ("share/morla_operator_control/config", glob("config/*")),
        ("share/morla_operator_control/launch", glob("launch/*launch.py")),
    ],
    install_requires=["setuptools"], zip_safe=True, license="MIT",
    maintainer="MORLA maintainers", maintainer_email="robot@example.com",
    description="Reusable operator command adapters",
    entry_points={"console_scripts": [
        "operator_router = morla_operator_control.router:main",
        "voice_input = morla_operator_control.voice:main",
        "pedal_input = morla_operator_control.pedal:main",
        "operator_feedback = morla_operator_control.feedback:main",
    ]},
)
