import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_mppi_nav')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params_real.yaml')
    default_path = os.path.expanduser('~/roboracer_ws/paths/track_real_center_open.csv')

    path_file_arg = DeclareLaunchArgument('path_file', default_value=default_path)
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)

    return LaunchDescription([
        path_file_arg,
        params_file_arg,
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['controller_server'],
            }],
        ),
        Node(
            package='f1tenth_mppi_nav',
            executable='cmd_vel_to_ackermann',
            name='cmd_vel_to_ackermann',
            output='screen',
            parameters=[{
                'wheelbase': 0.307,
                'max_steering_angle': 0.4189,
            }],
        ),
        Node(
            package='f1tenth_mppi_nav',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[{'path_file': LaunchConfiguration('path_file')}],
        ),
    ])
