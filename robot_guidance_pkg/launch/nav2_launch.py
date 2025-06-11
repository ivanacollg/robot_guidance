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

        # Map server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[param_file, {'yaml_filename': map_file}]
        ),

        # Nav2 Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[param_file, {'use_sim_time': use_sim_time}]
        ),

        # Nav2 Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[param_file, 
                        {'use_sim_time': use_sim_time,
                            'current_goal_checker': 'simple_goal_checker',
                            'current_progress_checker': 'simple_progress_checker'
                        }]
        ),

        # Nav2 Behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[param_file, {'use_sim_time': use_sim_time}]
        ),

        # Nav2 BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[param_file, {'use_sim_time': use_sim_time}]
        ),

        # Waypoint follower
        #Node(
        #    package='nav2_waypoint_follower',
        #    executable='waypoint_follower',
        #    name='waypoint_follower',
        #    output='screen',
        #    parameters=[param_file, {'use_sim_time': use_sim_time}]
        #),

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
                package='robot_guidance_pkg',
                executable='velocity_integrator',
                name='amcl',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'cmd_vel_topic': '/cmd_vel',
                    'odom_topic': '/odom',
                }]
            ),

            Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                name='camera',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'video_device': '/dev/video4',
                    'image_size': [1920, 1080],
                    'camera_frame_id': 'camera_frame',
                    'pixel_format': 'YUYV',  # or 'MJPG' depending on your camera
                    'io_method': 'mmap',
                    'framerate': 30,
                    'camera_info_url': ''  # Optional: Add path to your calibration .yaml file
                }],
                remappings=[
                    ('/image_raw', '/image_raw'),  # Optional: match expected topic names
                    ('/camera_info', '/camera_info')
                ]
            ),

            # April Tag Detection
            Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='apriltag_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_tag_detections': True,
                    'publish_tag_detections_image': True,
                    'publish_tf': True,
                    'camera_frame': 'camera_link',  
                    'tag_family': 'tag36h11',
                    'tag_size': 0.17,  # Tag size in meters
                    'approximate_sync': True,  
                }],
                remappings=[
                    ('image_rect', '/image_raw'),          # Adjust to your image topic
                    ('camera_info', '/camera_info'),       # Adjust to your camera_info
                    ('detections', '/tag_detections'),        # Output topic
                ],
                arguments=['--ros-args', '--log-level', 'error'],
            ),

            Node(
                package='robot_guidance_pkg',
                executable='tag_map_publisher',
                name='tag_map_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'tag_map_path': tag_map_file,
                }]
            ),

        ], condition=IfCondition(simple_simulation)),

        # RViz (conditionally launched)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_file],
            condition=IfCondition(use_rviz),
            output='screen'
        ),

                # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'bond_timeout': 0.0, # Fix to allow velocity integrator localization without bonding
                'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
            }]
        ),
    ])