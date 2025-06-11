import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ensure the node runs in your virtual environment
    env = os.environ.copy()
    env['PYTHONPATH'] = f"/home/ivana/ros2_venv/lib/python3.12/site-packages:{env.get('PYTHONPATH', '')}"
    # Parameter file path
    pkg_path = get_package_share_directory('robot_guidance_pkg')
    depth_control_param_file = os.path.join(pkg_path, 'config', 'depth_control_params.yaml')
    straffing_control_param_file = os.path.join(pkg_path, 'config', 'straffing_control_params.yaml')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = True

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to launch RViz'
        ),

        # USB Camera Node
        #Node(
        #    package='usb_cam',
        #    executable='usb_cam_node_exe',  # Adjust to actual executable name if needed
        #    name='usb_cam',
        #    output='screen',
        #    parameters=[{
        #        'video_device': '/dev/video4',
        #        'image_width': 1920,
        #        'image_height': 1080,
        #        'pixel_format': 'mjpeg2rgb',
        #        'camera_frame_id': 'usb_cam',
        #        'io_method': 'mmap',
        #    }],
        #    env=env
        #),

        # Rectifying Image Node
        #Node(
        #    package='image_proc',
        #    executable='rectify_node',
        #    name='camera',
        #    namespace='',  # IMPORTANT: must match camera name used in usb_cam's topics
        #    remappings=[
        #        ('image', '/image_raw'),
        #        ('camera_info', '/camera_info'),
        #    ],
        #    output='screen',
        #),


        Node(
            package='robot_guidance_pkg',
            executable='depth_control_server',
            name='depth_control_server',
            parameters=[
                depth_control_param_file,
                {
                    'cmd_vel_topic': '/cmd_vel',
                    'odom_topic': '/odom'
                }
            ],
            env=env
        ),

        Node(
            package='robot_guidance_pkg',
            executable='straffing_control_server',
            name='straffing_control_server',
            parameters=[
                straffing_control_param_file,
                {
                    'cmd_vel_topic': '/cmd_vel',
                    'odom_topic': '/odom'
                }
            ],
            env=env
        ),

        Node(
            package='robot_guidance_pkg',
            executable='apriltag_navigation_server',
            #name='apriltag_navigation_server', # FIx for not getting doubled nodes -> if not it renames the basic navigator 
            parameters=[
                {
                    'tag_detections_topic': '/tag_detections',
                    'odom_topic': '/odom'
                }
            ],
            env=env
        ),

        # RViz (conditionally launched)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            #arguments=['-d'],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
