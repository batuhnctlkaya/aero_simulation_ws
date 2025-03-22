from setuptools import setup
import os
from glob import glob

package_name = 'aero_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Aero Simulation Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_control_node = aero_simulation.joystick_control_node:main',
            'rover_control_node = aero_simulation.rover_control_node:main',
            'camera_node = aero_simulation.camera_node:main',
            'robot_state_publisher_node = aero_simulation.robot_state_publisher_node:main',
            'joint_state_publisher_node = aero_simulation.joint_state_publisher_node:main',
        ],
    },
)
