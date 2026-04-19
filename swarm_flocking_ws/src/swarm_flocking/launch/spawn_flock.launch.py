#!/usr/bin/env python3
# FILE: launch/spawn_flock.launch.py
"""
spawn_flock.launch.py — Spawn N TurtleBot3 robots in a running Gazebo session
and launch their boid_node instances.

Expects Gazebo to already be running.

Usage:
    ros2 launch swarm_flocking spawn_flock.launch.py num_robots:=6
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='6',
        description='Number of TurtleBot3 robots to spawn')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    tb3_model_arg = DeclareLaunchArgument(
        'turtlebot3_model', default_value='burger',
        description='TurtleBot3 model type')

    spawn_action = OpaqueFunction(function=_do_spawn)

    return LaunchDescription([
        num_robots_arg,
        use_sim_time_arg,
        tb3_model_arg,
        spawn_action,
    ])


def _do_spawn(context, *args, **kwargs):
    pkg_flocking = get_package_share_directory('swarm_flocking')
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    params_file  = os.path.join(pkg_flocking, 'config', 'flocking_params.yaml')

    num_robots   = int(context.launch_configurations.get('num_robots', '6'))
    use_sim_time = context.launch_configurations.get('use_sim_time', 'true')
    tb3_model    = context.launch_configurations.get('turtlebot3_model', 'burger')

    # URDF for robot_state_publisher (TF only, no Gazebo plugins)
    urdf_path = os.path.join(
        pkg_tb3_desc, 'urdf', f'turtlebot3_{tb3_model}.urdf')
    try:
        with open(urdf_path, 'r') as f:
            robot_description = f.read()
    except FileNotFoundError:
        robot_description = (
            '<?xml version="1.0"?>'
            '<robot name="turtlebot3_burger"><link name="base_link"/></robot>'
        )

    # SDF model (HAS Gazebo plugins: diff_drive, lidar, IMU)
    sdf_path = os.path.join(
        pkg_tb3_gazebo, 'models', f'turtlebot3_{tb3_model}', 'model.sdf')

    # Grid layout: 3 columns, starting at (2.0, 4.5)
    start_x, start_y, spacing = 2.0, 4.5, 0.7

    actions = []
    for i in range(num_robots):
        row = i // 3
        col = i % 3
        x = start_x + col * spacing
        y = start_y + row * spacing
        ns = f'robot_{i}'

        GAZEBO_READY_DELAY = 30.0
        spawn_time = GAZEBO_READY_DELAY + i * 1.5
        boid_time  = spawn_time + 5.0

        # robot_state_publisher — TF frames (URDF, no plugins)
        rsp_action = TimerAction(
            period=max(0.5, float(GAZEBO_READY_DELAY - 1.5)),
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    namespace=ns,
                    parameters=[{
                        'robot_description': robot_description,
                        'use_sim_time': use_sim_time == 'true',
                        'frame_prefix': f'{ns}/',
                    }],
                    output='screen',
                ),
            ],
        )

        # Spawn from SDF (HAS Gazebo plugins for odom/scan/cmd_vel)
        spawn_action = TimerAction(
            period=float(spawn_time),
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', ns,
                        '-file', sdf_path,
                        '-x', str(x),
                        '-y', str(y),
                        '-z', '0.01',
                        '-robot_namespace', f'/{ns}',
                    ],
                    output='screen',
                ),
            ],
        )

        # boid_node — fires after spawn completes
        boid_action = TimerAction(
            period=float(boid_time),
            actions=[
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
                            'spawn_x': x,
                            'spawn_y': y,
                            'use_sim_time': use_sim_time == 'true',
                        },
                    ],
                    output='screen',
                ),
            ],
        )

        actions.extend([rsp_action, spawn_action, boid_action])

    # Single flock_monitor instance
    monitor = Node(
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
    actions.append(monitor)

    return actions
