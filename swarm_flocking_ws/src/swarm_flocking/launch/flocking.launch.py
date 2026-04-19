#!/usr/bin/env python3
# FILE: launch/flocking.launch.py
"""
flocking.launch.py — Launch only the boid_node instances and the flock monitor.

Useful when robots are already spawned and you only want to start/restart
the flocking behavior (e.g., for parameter tuning without restarting Gazebo).

Usage:
    ros2 launch swarm_flocking flocking.launch.py num_robots:=6
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():
    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='6',
        description='Number of robots (must match what is spawned in Gazebo)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    return LaunchDescription([
        num_robots_arg,
        use_sim_time_arg,
        OpaqueFunction(function=_launch_boids),
    ])


def _launch_boids(context, *args, **kwargs):
    pkg_flocking = get_package_share_directory('swarm_flocking')
    params_file  = os.path.join(pkg_flocking, 'config', 'flocking_params.yaml')

    num_robots   = int(context.launch_configurations.get('num_robots', '6'))
    use_sim_time = context.launch_configurations.get('use_sim_time', 'true')

    actions = []
    for i in range(num_robots):
        ns = f'robot_{i}'
        actions.append(
            Node(
                package='swarm_flocking',
                executable='boid_node',
                name=f'boid_{i}',
                namespace=ns,
                parameters=[
                    params_file,
                    {
                        'robot_id': i,
                        'num_robots': num_robots,
                        'use_sim_time': use_sim_time == 'true',
                    },
                ],
                output='screen',
            )
        )

    # Flock monitor
    actions.append(
        Node(
            package='swarm_flocking',
            executable='flock_monitor_node',
            name='flock_monitor',
            parameters=[
                params_file,
                {
                    'num_robots': num_robots,
                    'use_sim_time': use_sim_time == 'true',
                },
            ],
            output='screen',
        )
    )

    return actions
