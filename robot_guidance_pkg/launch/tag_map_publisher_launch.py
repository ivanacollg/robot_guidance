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
    tag_map_file = os.path.join(pkg_path, 'config', 'tag_map.yaml')
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

        

        # --- Simple Simulation Nodes ---
        GroupAction([
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_map_odom',
                arguments=[
                    '--x', '0.0',
                    '--y', '0.0',
                    '--z', '0.0',
                    '--roll', '0.0',
                    '--pitch', '0.0',
                    '--yaw', '0.0',
                    '--frame-id', 'map',
                    '--child-frame-id', 'odom'
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_base_camera',
                arguments=[
                    '--x', '0.0',
                    '--y', '0.0',
                    '--z', '0.0',
                    '--roll', '-1.5708 ',
                    '--pitch', '0.0',
                    '--yaw', '-1.5708 ',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'camera_frame'
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_base_camera',
                arguments=[
                    '--x', '0.0',
                    '--y', '0.0',
                    '--z', '0.0',
                    '--roll', '1.5708',
                    '--pitch', '0.0',
                    '--yaw', '1.5708 ',
                    '--frame-id', 'map',
                    '--child-frame-id', 'optitrack'
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),

            # April Tag Detection
            Node(
                package='robot_guidance_pkg',
                executable='tag_map_publisher',
                name='tag_map_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'tag_map_path': tag_map_file,
                }],
            ),


        ], condition=IfCondition(simple_simulation)),

    ])