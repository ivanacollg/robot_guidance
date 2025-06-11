import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ensure the node runs in your virtual environment
    env = os.environ.copy()
    env['PYTHONPATH'] = f"/home/ivana/ros2_venv/lib/python3.12/site-packages:{env.get('PYTHONPATH', '')}"
    # Parameter file path
    pkg_path = get_package_share_directory('robot_guidance_pkg')
    param_file = os.path.join(pkg_path, 'config', 'waypoint_follower_params.yaml')

    return LaunchDescription([
        # Node for waypoint_follower with parameters and namespace
        Node(
            package='robot_guidance_pkg',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace='waypoint_follower',
            output='screen',
            parameters=[param_file],
            env=env
        ),

        Node(
            package='robot_guidance_pkg',
            executable='velocity_integrator',
            name='velocity_integrator',
            output='screen',
        ),
    ])
