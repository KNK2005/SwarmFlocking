#!/usr/bin/env python3
"""
portable_sim.launch.py — Cross-distro simulation entry point.

Backends:
  - headless (default): works on Humble/Jazzy and does not require Gazebo.
  - gazebo: includes full_sim.launch.py (Gazebo + spawn + boids + RViz).

Examples:
  ros2 launch swarm_flocking portable_sim.launch.py
    ros2 launch swarm_flocking portable_sim.launch.py backend:=gazebo gazebo_flavor:=harmonic num_robots:=6 world_name:=open_field
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='headless',
            description='Simulation backend: headless or gazebo'),
        DeclareLaunchArgument(
            'num_robots', default_value='6',
            description='Number of robots in simulation'),
        DeclareLaunchArgument(
            'world_name', default_value='open_field',
            description='World basename used by gazebo backend'),
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='When using gazebo backend, run server only (no GUI window)'),
        DeclareLaunchArgument(
            'gazebo_flavor', default_value='auto',
            description='Gazebo backend flavor: auto, harmonic, or classic'),
        DeclareLaunchArgument(
            'odom_is_local', default_value='false',
            description='When using Harmonic, interpret odom as local and apply spawn offsets'),
        DeclareLaunchArgument(
            'spawn_origin_x', default_value='4.0',
            description='Spawn grid origin X used by Harmonic backend'),
        DeclareLaunchArgument(
            'spawn_origin_y', default_value='6.0',
            description='Spawn grid origin Y used by Harmonic backend'),
        DeclareLaunchArgument(
            'spawn_spacing_x', default_value='1.0',
            description='Spawn grid spacing in X used by Harmonic backend'),
        DeclareLaunchArgument(
            'spawn_spacing_y', default_value='1.0',
            description='Spawn grid spacing in Y used by Harmonic backend'),
        DeclareLaunchArgument(
            'spawn_columns', default_value='3',
            description='Spawn grid columns used by Harmonic backend'),
        DeclareLaunchArgument(
            'dt', default_value='0.1',
            description='Physics timestep for headless backend'),
        DeclareLaunchArgument(
            'enable_rviz', default_value='false',
            description='Launch RViz when running headless backend'),
        DeclareLaunchArgument(
            'enable_obstacle_avoidance', default_value='false',
            description='Enable boid obstacle avoidance logic'),
        DeclareLaunchArgument(
            'success_timeout_s', default_value='300.0',
            description='Monitor timeout in seconds before completion'),
        DeclareLaunchArgument(
            'auto_shutdown_on_completion', default_value='true',
            description='Auto-stop nodes when monitor completes'),
        DeclareLaunchArgument(
            'waypoints', default_value='[6.0, 6.5, 9.0, 7.6, 12.0, 8.6, 15.0, 9.4, 17.0, 10.0]',
            description='Flat waypoint list [x0,y0,x1,y1,...] for headless backend'),
        DeclareLaunchArgument(
            'spawn_coords',
            default_value='[-2.0, -0.5, -2.0, 0.5, -3.0, -0.5, -3.0, 0.5, -4.0, -0.5, -4.0, 0.5]',
            description='Flat spawn offsets [x0,y0,x1,y1,...] for headless backend'),
        OpaqueFunction(function=_dispatch_backend),
    ])


def _dispatch_backend(context, *args, **kwargs):
    pkg_flocking = get_package_share_directory('swarm_flocking')
    backend = context.launch_configurations.get('backend', 'headless').strip().lower()
    gazebo_flavor = context.launch_configurations.get('gazebo_flavor', 'auto').strip().lower()

    def _headless_include(force_rviz: bool = False):
        rviz_arg = context.launch_configurations.get('enable_rviz', 'false')
        if force_rviz and rviz_arg.strip() == 'false':
            rviz_arg = 'true'

        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_flocking, 'launch', 'headless_sim.launch.py')
            ),
            launch_arguments={
                'num_robots': context.launch_configurations.get('num_robots', '6'),
                'use_sim_time': 'false',
                'dt': context.launch_configurations.get('dt', '0.1'),
                'enable_rviz': rviz_arg,
                'success_timeout_s': context.launch_configurations.get('success_timeout_s', '300.0'),
                'auto_shutdown_on_completion': context.launch_configurations.get('auto_shutdown_on_completion', 'true'),
                'waypoints': context.launch_configurations.get('waypoints', ''),
                'spawn_coords': context.launch_configurations.get('spawn_coords', ''),
            }.items(),
        )

    if backend == 'gazebo':
        if gazebo_flavor not in ('auto', 'harmonic', 'classic'):
            raise RuntimeError(
                f"Unsupported gazebo_flavor '{gazebo_flavor}'. Expected auto, harmonic, or classic."
            )

        harmonic_missing = []
        classic_missing = []

        for pkg in ('ros_gz_sim', 'ros_gz_bridge'):
            try:
                get_package_share_directory(pkg)
            except Exception:
                harmonic_missing.append(pkg)

        for pkg in ('gazebo_ros', 'turtlebot3_gazebo', 'turtlebot3_description'):
            try:
                get_package_share_directory(pkg)
            except Exception:
                classic_missing.append(pkg)

        harmonic_ready = len(harmonic_missing) == 0
        classic_ready = len(classic_missing) == 0

        if gazebo_flavor in ('auto', 'harmonic') and harmonic_ready:
            return [
                LogInfo(msg='[portable_sim] Launching Gazebo Harmonic backend (ros_gz).'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_flocking, 'launch', 'full_sim_harmonic.launch.py')
                    ),
                    launch_arguments={
                        'num_robots': context.launch_configurations.get('num_robots', '6'),
                        'world_name': context.launch_configurations.get('world_name', 'open_field'),
                        # Harmonic can run without bridged /clock; keep ROS timers alive.
                        'use_sim_time': 'false',
                        'odom_is_local': context.launch_configurations.get('odom_is_local', 'false'),
                        'headless': context.launch_configurations.get('headless', 'true'),
                        'enable_rviz': context.launch_configurations.get('enable_rviz', 'false'),
                        'enable_obstacle_avoidance': context.launch_configurations.get('enable_obstacle_avoidance', 'false'),
                        'waypoints': context.launch_configurations.get('waypoints', ''),
                        'spawn_origin_x': context.launch_configurations.get('spawn_origin_x', '4.0'),
                        'spawn_origin_y': context.launch_configurations.get('spawn_origin_y', '6.0'),
                        'spawn_spacing_x': context.launch_configurations.get('spawn_spacing_x', '1.0'),
                        'spawn_spacing_y': context.launch_configurations.get('spawn_spacing_y', '1.0'),
                        'spawn_columns': context.launch_configurations.get('spawn_columns', '3'),
                    }.items(),
                ),
            ]

        if gazebo_flavor in ('auto', 'classic') and classic_ready:
            return [
                LogInfo(msg='[portable_sim] Launching Gazebo Classic backend (gazebo_ros).'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_flocking, 'launch', 'full_sim.launch.py')
                    ),
                    launch_arguments={
                        'num_robots': context.launch_configurations.get('num_robots', '6'),
                        'world_name': context.launch_configurations.get('world_name', 'open_field'),
                        'use_sim_time': 'true',
                    }.items(),
                ),
            ]

        msg = (
            '[portable_sim] backend:=gazebo requested, but no supported Gazebo stack is fully available. '
            f'harmonic missing={harmonic_missing}, classic missing={classic_missing}. '
            'Falling back to backend:=headless.'
        )
        return [
            LogInfo(msg=msg),
            _headless_include(force_rviz=True),
        ]

    if backend == 'headless':
        return [_headless_include()]

    raise RuntimeError(
        f"Unsupported backend '{backend}'. Expected 'headless' or 'gazebo'."
    )
