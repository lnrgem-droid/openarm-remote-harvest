from glob import glob
from os.path import join
from setuptools import find_packages, setup

package_name = "remote_teleop_runtime"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (join("share", package_name, "config"), glob("config/*.yaml")),
        (join("share", package_name, "script"), glob("script/*.sh")),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openarm",
    maintainer_email="openarm@todo.todo",
    description="OpenArm remote teleoperation runtime",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "remote-teleop-leader = remote_teleop_runtime.leader:main",
        "remote-teleop-follower = remote_teleop_runtime.follower:main",
        "remote-teleop-control = remote_teleop_runtime.control:main",
    ]},
)
