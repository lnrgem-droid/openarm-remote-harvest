from glob import glob
from os.path import join

from setuptools import find_packages, setup


package_name = "remote_teleop_follower_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openarm",
    maintainer_email="openarm@todo.todo",
    description="Jetson-local follower safety state machine and watchdog",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "remote-teleop-follower-watchdog = remote_teleop_follower_safety.service:main",
            "remote-teleop-follower-safety-sim = remote_teleop_follower_safety.simulation:main",
        ],
    },
)
