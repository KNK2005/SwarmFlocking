#!/usr/bin/env python3
# FILE: swarm_flocking/utils/obstacle_avoidance.py
"""
Obstacle avoidance utility — converts sensor_msgs/LaserScan into a
repulsive 2-D force vector (world frame).

The algorithm:
  1. Iterate over all laser rays whose range < obstacle_threshold.
  2. For each such ray, compute the obstacle position in the robot frame.
  3. Accumulate a repulsive force pointing AWAY from the obstacle, weighted
     by 1/dist² (closer obstacles push harder).  The force is also scaled
     by a soft ramp that fades gracefully between range_min and threshold.
  4. Transform the accumulated robot-frame force to the world frame using
     the robot's current yaw angle.
  5. Return a normalised world-frame vector.  If no obstacles are detected
     within the threshold, returns (0.0, 0.0).
"""

import math
from typing import Tuple, Optional

from sensor_msgs.msg import LaserScan

Vec2 = Tuple[float, float]

# Maximum number of rays processed per cycle to bound CPU time on dense scans
MAX_RAYS_PROCESSED = 360


def laser_to_repulsive_force(
    scan: Optional[LaserScan],
    robot_theta: float,
    obstacle_threshold: float,
) -> Vec2:
    """
    Compute an obstacle-repulsion force vector in the **world frame**.

    Parameters
    ----------
    scan : LaserScan or None
        The latest laser scan message.  If None, returns (0, 0).
    robot_theta : float
        Current robot yaw (radians, world frame).
    obstacle_threshold : float
        Range (meters) below which a laser reading triggers repulsion.

    Returns
    -------
    (fx_world, fy_world) : Vec2
        Normalised 2-D repulsive force in the world frame.
    """
    if scan is None:
        return (0.0, 0.0)

    fx_robot = 0.0
    fy_robot = 0.0
    hit_count = 0

    # Stride to limit processing cost on high-resolution scans
    total_rays = len(scan.ranges)
    stride = max(1, total_rays // MAX_RAYS_PROCESSED)

    for i in range(0, total_rays, stride):
        dist = scan.ranges[i]

        # Skip invalid / out-of-range readings
        if not math.isfinite(dist):
            continue
        if dist < scan.range_min or dist >= obstacle_threshold:
            continue

        # Angle of this ray in the robot frame
        angle = scan.angle_min + i * scan.angle_increment

        # Obstacle position in robot frame
        obs_x = dist * math.cos(angle)
        obs_y = dist * math.sin(angle)

        # Soft ramp: force fades as distance approaches the threshold
        # This prevents sudden jumps when an obstacle enters the zone.
        proximity_factor = (obstacle_threshold - dist) / obstacle_threshold

        # Inverse-square repulsive weight; clamp to avoid blow-up at range_min
        safe_dist = max(dist, scan.range_min + 1e-3)
        weight = proximity_factor / (safe_dist * safe_dist)

        # Repulsive direction: AWAY from the obstacle
        fx_robot -= obs_x * weight
        fy_robot -= obs_y * weight
        hit_count += 1

    if hit_count == 0:
        return (0.0, 0.0)

    # Average across contributing rays
    fx_robot /= hit_count
    fy_robot /= hit_count

    # Transform from robot frame to world frame
    cos_t = math.cos(robot_theta)
    sin_t = math.sin(robot_theta)
    fx_world = fx_robot * cos_t - fy_robot * sin_t
    fy_world = fx_robot * sin_t + fy_robot * cos_t

    # Normalise
    mag = math.hypot(fx_world, fy_world)
    if mag < 1e-6:
        return (0.0, 0.0)
    return (fx_world / mag, fy_world / mag)
