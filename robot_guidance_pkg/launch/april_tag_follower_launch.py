import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ensure the node runs in your virtual environment
    env = os.environ.copy()
    env['PYTHONPATH'] = f"/home/ivana/ros2_venv/lib/python3.12/site-packages:{env.get('PYTHONPATH', '')}"

    pkg_share = get_package_share_directory('robot_guidance_pkg')
    rviz_default_config = os.path.join(pkg_share, 'rviz', 'apriltag.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),
        DeclareLaunchArgument(
            'raw_img_topic',
            default_value='image_raw',
            description='Topic name for raw image input'
        ),
        # Optional: Declare path to RViz config file
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_default_config,
            description='Full path to RViz config file'
        ),

        # USB Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',  # Adjust to actual executable name if needed
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video4',
                'image_width': 1920,
                'image_height': 1080,
                'pixel_format': 'mjpeg2rgb',
                'camera_frame_id': 'usb_cam',
                'io_method': 'mmap',
            }],
            env=env
        ),

        # AprilTag follower node
        Node(
            package='robot_guidance_pkg',
            executable='apriltag_follower',
            name='apriltag_follower_node',
            output='screen',
            parameters=[{
                'image_sub': LaunchConfiguration('raw_img_topic')
            }],
            env=env
        ),

        # RViz (conditionally launched)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
            env=env
        ),

    ])
