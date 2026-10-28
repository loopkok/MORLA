from setuptools import find_packages, setup
setup(name="morla_learning", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/morla_learning"]),
                  ("share/morla_learning", ["package.xml"])],
      install_requires=["setuptools"],
      entry_points={"console_scripts": ["morla-learning = morla_learning.cli:main"]})
