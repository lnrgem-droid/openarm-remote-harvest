from glob import glob
from os.path import join

from setuptools import find_packages, setup


package_name = "remote_teleop_protocol"

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
    description="Hardware-independent OpenArm remote teleoperation protocol",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "remote-teleop-sim-leader = remote_teleop_protocol.simulation:leader_main",
            "remote-teleop-sim-follower = remote_teleop_protocol.simulation:follower_main",
        ],
    },
)
