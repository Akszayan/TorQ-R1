import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'torq_r1_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akszayan',
    maintainer_email='akszayan@todo.todo',
    description='SLAM, Navigation, Path Publishing and Teleoperation modules for TorQ-R1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arrow_teleop_node = torq_r1_navigation.arrow_teleop_node:main',
            'path_publisher_node = torq_r1_navigation.path_publisher_node:main',
            'obstacle_avoidance_node = torq_r1_navigation.obstacle_avoidance_node:main',
            'telemetry_node = torq_r1_navigation.telemetry_node:main',
            'autonomous_explorer_node = torq_r1_navigation.autonomous_explorer_node:main',
        ],
    },
)
