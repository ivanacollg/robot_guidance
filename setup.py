from setuptools import setup
import os
from glob import glob

package_name = 'robot_guidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Optionally: install rviz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'dt-apriltags'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='AprilTag follower node for ROS 2 robot guidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_follower = robot_guidance.apriltag_follower:main',
            'waypoint_follower = robot_guidance.waypoint_follower:main',
            'velocity_integrator = robot_guidance.velocity_integrator:main',
            'depth_control_server = robot_guidance.depth_control_server:main',
            'apriltag_navigator_server = robot_guidance.apriltag_navigator_server:main',
        ],
    },
)