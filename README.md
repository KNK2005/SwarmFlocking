# Swarm Flocking with Obstacle Navigation

A ROS 2 multi-robot flocking system (Boids/Reynolds model) with obstacle avoidance,
waypoint migration, experiment metrics, and headless simulation support.

Supported setups:
- Ubuntu 24.04 + ROS 2 Jazzy (recommended)
- Ubuntu 22.04 + ROS 2 Humble

## Overview

Main components:
- Per-robot boid control node with separation, alignment, cohesion, obstacle avoidance, and migration
- Flock monitor node with centroid, cohesion, split detection, collision metrics, and success criteria classification
- Headless physics node (fast, no Gazebo required)
- Optional Gazebo-backed simulation path

## Ubuntu 24.04 Quick Start (Recommended)

### 1. Install ROS 2 Jazzy and tools

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-rmw-fastrtps-cpp \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  git
```

Optional GUI simulator backend on Jazzy:

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-sim
```

### 2. Initialize rosdep

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

### 3. Clone and build

```bash
cd ~
git clone <your-repo-url>.git
cd swarm_flocking_ws/swarm_flocking_ws

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-select swarm_interfaces swarm_flocking swarm_flocking_gazebo

source install/setup.bash
```

### 4. Run simulation

Headless (recommended, no Gazebo required):

```bash
source /opt/ros/jazzy/setup.bash
source ~/swarm_flocking_ws/swarm_flocking_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

ros2 launch swarm_flocking portable_sim.launch.py backend:=headless num_robots:=6
```

Gazebo backend (if installed):

```bash
ros2 launch swarm_flocking portable_sim.launch.py \
  backend:=gazebo num_robots:=6 world_name:=obstacle_course
```

## Monitoring and Tuning

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/swarm_flocking_ws/swarm_flocking_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
```

Useful commands:

```bash
ros2 topic echo /flock/state
ros2 topic echo /robot_0/cmd_vel
ros2 topic list
ros2 node list
```

Live tuning (takes effect immediately):

```bash
ros2 param set /boid_node_0 w_separation 3.0
ros2 param set /boid_node_0 w_cohesion 2.0
ros2 param set /boid_node_0 w_migration 0.8
ros2 param set /boid_node_0 w_obstacle 4.0
```

Note:
- Exact node names can differ by launch path/namespace. Use `ros2 node list` to confirm.

## Troubleshooting

- DDS communication issues between terminals:
  - Ensure both terminals use identical `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID`.
- GUI simulator missing on Ubuntu 24.04:
  - Use headless backend (`backend:=headless`), or install `ros_gz_sim` packages.
- No robot motion:
  - Check `/robot_0/odom` publisher count and `/flock/state` output.

## Upload Readiness (GitHub)

This repository is prepared for upload with:
- Updated Ubuntu 24.04/Jazzy instructions
- Portable launch entrypoint for headless and Gazebo backends
- Repository `.gitignore` rules for ROS/colcon artifacts
- Top-level Apache-2.0 license file

Before pushing:

```bash
git status
git add .
git commit -m "Prepare Ubuntu 24.04 support and GitHub-ready repo"
git remote add origin <your-github-repo-url>
git push -u origin main
```

## License

Apache-2.0. See LICENSE.
