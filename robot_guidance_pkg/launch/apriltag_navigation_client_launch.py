import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    simple_simulation = LaunchConfiguration('simple_simulation')
    use_rviz = LaunchConfiguration('use_rviz')

    # Parameter file path
    pkg_path = get_package_share_directory('robot_guidance_pkg')
    map_file = os.path.join(pkg_path, 'maps', 'empty_map.yaml')
    param_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    tag_map_file = os.path.join(pkg_path, 'config', 'poses_map.yaml')
    rviz_file = os.path.join(pkg_path, 'rviz', 'nav2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (true for Gazebo or fake clock)'
        ),

        DeclareLaunchArgument(
            'simple_simulation',
            default_value='true',
            description='Launch simple simulation nodes like velocity integrator and static transform'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),



            Node(
                package='robot_guidance_pkg',
                executable='apriltag_navigation_client',
                name='apriltag_navigation_client',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'tag_map_path': tag_map_file,
                }]
            ),



    ])