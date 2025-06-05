import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ensure the node runs in your virtual environment
    env = os.environ.copy()
    env['PYTHONPATH'] = f"/home/ivana/ros2_venv/lib/python3.12/site-packages:{env.get('PYTHONPATH', '')}"
    # Parameter file path
    pkg_path = get_package_share_directory('robot_guidance_pkg')
    #param_file = os.path.join(pkg_path, 'config', 'depth_control_params.yaml')

    return LaunchDescription([

        Node(
            package='robot_guidance_pkg',
            executable='apriltag_navigation_server',
            name='apriltag_navigation_server',
            parameters=[
                {
                    'tag_detections_topic': '/tag_detections',
                    'odom_topic': '/odom'
                }
            ],
            env=env
        )
    ])
