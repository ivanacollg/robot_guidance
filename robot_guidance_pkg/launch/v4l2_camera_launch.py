from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (true for Gazebo or fake clock)'
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
    ])