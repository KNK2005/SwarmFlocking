#!/usr/bin/env python3
# FILE: swarm_flocking_gazebo/launch/gazebo_world.launch.py
"""
gazebo_world.launch.py — Launch the requested world on whichever simulator is
available in the current ROS distro.

Priority order:
    1) gazebo_ros (Gazebo Classic, typical on Humble/Ubuntu 22.04)
    2) ros_gz_sim (Gazebo Harmonic/Ignition stack, typical on Jazzy/Ubuntu 24.04)

Usage (standalone):
        ros2 launch swarm_flocking_gazebo gazebo_world.launch.py

Or included from full_sim.launch.py with the 'world' argument overridden.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gz_world = get_package_share_directory('swarm_flocking_gazebo')

    default_world = os.path.join(
        pkg_gz_world, 'worlds', 'obstacle_course.world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the Gazebo world file to load')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Set to true for verbose Gazebo output')

    actions = [
        world_arg,
        use_sim_time_arg,
        verbose_arg,
    ]

    # Prefer Gazebo Classic launch if available.
    try:
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

        gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'verbose': LaunchConfiguration('verbose'),
            }.items())

        gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            launch_arguments={
                'verbose': LaunchConfiguration('verbose'),
            }.items())

        actions.extend([gzserver, gzclient])
        return LaunchDescription(actions)
    except Exception:
        pass

    # Fallback for Ubuntu 24 / ROS 2 Jazzy environments.
    try:
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r ', LaunchConfiguration('world')],
            }.items(),
        )
        actions.append(gz_sim)
        return LaunchDescription(actions)
    except Exception as exc:
        raise RuntimeError(
            "No supported simulator backend found. Install either 'gazebo_ros' "
            "(Gazebo Classic) or 'ros_gz_sim' (Gazebo Harmonic)."
        ) from exc
