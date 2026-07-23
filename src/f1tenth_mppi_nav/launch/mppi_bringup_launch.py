import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_mppi_nav')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # map_server is already brought up by f1tenth_gym_ros's own
    # gym_bridge_launch.py (its own lifecycle_manager_localization) -
    # reuse that instead of running a second one under the same node name.
    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
        Node(
            package='f1tenth_mppi_nav',
            executable='cmd_vel_to_ackermann',
            name='cmd_vel_to_ackermann',
            output='screen',
        ),
        Node(
            package='f1tenth_mppi_nav',
            executable='goal_pose_relay',
            name='goal_pose_relay',
            output='screen',
        ),
    ])
