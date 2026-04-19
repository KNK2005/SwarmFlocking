#!/usr/bin/env python3
"""
portable_sim.launch.py — Cross-distro simulation entry point.

Backends:
  - headless (default): works on Humble/Jazzy and does not require Gazebo.
  - gazebo: includes full_sim.launch.py (Gazebo + spawn + boids + RViz).

Examples:
  ros2 launch swarm_flocking portable_sim.launch.py
  ros2 launch swarm_flocking portable_sim.launch.py backend:=gazebo num_robots:=6 world_name:=open_field
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
            'world_name', default_value='obstacle_course',
            description='World basename used by gazebo backend'),
        DeclareLaunchArgument(
            'dt', default_value='0.1',
            description='Physics timestep for headless backend'),
        DeclareLaunchArgument(
            'waypoints', default_value='[8.0, 7.5, 15.0, 7.5, 22.0, 7.5, 28.0, 7.5]',
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

    if backend == 'gazebo':
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_flocking, 'launch', 'full_sim.launch.py')
                ),
                launch_arguments={
                    'num_robots': context.launch_configurations.get('num_robots', '6'),
                    'world_name': context.launch_configurations.get('world_name', 'obstacle_course'),
                    'use_sim_time': 'true',
                }.items(),
            )
        ]

    if backend == 'headless':
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_flocking, 'launch', 'headless_sim.launch.py')
                ),
                launch_arguments={
                    'num_robots': context.launch_configurations.get('num_robots', '6'),
                    'use_sim_time': 'false',
                    'dt': context.launch_configurations.get('dt', '0.1'),
                    'waypoints': context.launch_configurations.get('waypoints', ''),
                    'spawn_coords': context.launch_configurations.get('spawn_coords', ''),
                }.items(),
            )
        ]

    raise RuntimeError(
        f"Unsupported backend '{backend}'. Expected 'headless' or 'gazebo'."
    )
