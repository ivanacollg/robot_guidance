import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ensure the node runs in your virtual environment
    env = os.environ.copy()
    env['PYTHONPATH'] = f"/home/ivana/ros2_venv/lib/python3.12/site-packages:{env.get('PYTHONPATH', '')}"
    # Parameter file path
    pkg_path = get_package_share_directory('robot_guidance_pkg')
    param_file = os.path.join(pkg_path, 'config', 'depth_control_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (true for Gazebo or fake clock)'
        ),

        Node(
            package='robot_guidance_pkg',
            executable='depth_control_server',
            name='depth_control_server',
            parameters=[
                param_file,
                {
                    'cmd_vel_topic': '/cmd_vel',
                    'odom_topic': '/odom'
                }
            ],
            env=env
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
                'node_names': ['amcl']
            }]
        ),
    ])
