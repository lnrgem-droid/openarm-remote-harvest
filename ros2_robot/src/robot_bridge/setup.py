from setuptools import setup
import os
from glob import glob

package_name = 'robot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'schemas'),
            glob('config/schemas/*.json')),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='openarm',
    maintainer_email='openarm@todo.todo',
    description='WebSocket + JSON 桥接平台，把 ROS2 接口暴露给外部算法调用',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = robot_bridge.bridge_node:main',
        ],
    },
)
