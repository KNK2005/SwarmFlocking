#!/usr/bin/env python3
# FILE: swarm_flocking/utils/reynolds.py
"""
Reynolds flocking math utilities.

Pure, stateless functions for computing the three classic boid rules
(separation, alignment, cohesion) plus helper geometry functions.
All forces are returned as normalised (unit) 2-D vectors (fx, fy),
so the caller controls weighting.

Neighbour tuple format used throughout:
    (robot_id: int, nx: float, ny: float, vx: float, vy: float, dist: float)
"""

import math
from typing import List, Tuple

# Type aliases for clarity
Vec2 = Tuple[float, float]
NeighbourEntry = Tuple[int, float, float, float, float, float]  # id,x,y,vx,vy,dist

# ---------------------------------------------------------------------------
# Generic vector helpers
# ---------------------------------------------------------------------------

def normalize(x: float, y: float, epsilon: float = 1e-6) -> Vec2:
    """Safely normalize a 2-D vector.  Returns (0, 0) for near-zero vectors."""
    mag = math.hypot(x, y)
    if mag < epsilon:
        return (0.0, 0.0)
    return (x / mag, y / mag)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp *value* to [min_val, max_val]."""
    return max(min_val, min(max_val, value))


def normalize_angle(angle: float) -> float:
    """Wrap *angle* (radians) into [-π, π]."""
    # Use modulo for speed; handles large values cleanly
    angle = math.fmod(angle + math.pi, 2.0 * math.pi)
    if angle < 0.0:
        angle += 2.0 * math.pi
    return angle - math.pi


def yaw_from_quaternion(q) -> float:
    """Extract yaw (rotation about Z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Reynolds rules
# ---------------------------------------------------------------------------

def compute_separation(
    my_x: float,
    my_y: float,
    neighbours: List[NeighbourEntry],
    separation_radius: float,
) -> Vec2:
    """
    SEPARATION — steer away from robots that are within *separation_radius*.

    For each neighbour inside the separation zone we accumulate a repulsive
    vector scaled by 1/dist² (closer neighbours push harder).  The average
    is taken then normalized to produce a unit vector.

    Returns (0, 0) if no neighbours are within the separation zone.
    """
    fx, fy = 0.0, 0.0
    count = 0

    for (_rid, nx, ny, _vx, _vy, dist) in neighbours:
        if dist < separation_radius and dist > 1e-6:
            # Vector pointing AWAY from neighbour; weight by inverse square
            weight = 1.0 / (dist * dist)
            fx += (my_x - nx) * weight
            fy += (my_y - ny) * weight
            count += 1

    if count == 0:
        return (0.0, 0.0)

    fx /= count
    fy /= count
    return normalize(fx, fy)


def compute_alignment(
    my_vx: float,
    my_vy: float,
    neighbours: List[NeighbourEntry],
) -> Vec2:
    """
    ALIGNMENT — steer to match the average velocity of neighbours.

    We compute the average neighbour velocity and subtract our own velocity
    to get the 'correction' needed to match the flock's heading.

    Returns (0, 0) if no neighbours exist.
    """
    if not neighbours:
        return (0.0, 0.0)

    n = len(neighbours)
    avg_vx = sum(vx for (_rid, _nx, _ny, vx, _vy, _dist) in neighbours) / n
    avg_vy = sum(vy for (_rid, _nx, _ny, _vx, vy, _dist) in neighbours) / n

    fx = avg_vx - my_vx
    fy = avg_vy - my_vy
    return normalize(fx, fy)


def compute_cohesion(
    my_x: float,
    my_y: float,
    neighbours: List[NeighbourEntry],
) -> Vec2:
    """
    COHESION — steer toward the centroid of neighbours.

    Average position of all neighbours gives the local 'center of mass'.
    We return a normalized vector pointing from our position to that center.

    Returns (0, 0) if no neighbours exist.
    """
    if not neighbours:
        return (0.0, 0.0)

    n = len(neighbours)
    cx = sum(nx for (_rid, nx, _ny, _vx, _vy, _dist) in neighbours) / n
    cy = sum(ny for (_rid, _nx, ny, _vx, _vy, _dist) in neighbours) / n

    return normalize(cx - my_x, cy - my_y)


# ---------------------------------------------------------------------------
# Migration (goal-seeking)
# ---------------------------------------------------------------------------

def compute_migration(
    my_x: float,
    my_y: float,
    goal_x: float,
    goal_y: float,
) -> Vec2:
    """
    MIGRATION — gentle pull toward a waypoint.

    Returns a normalized direction vector toward (goal_x, goal_y).
    Returns (0, 0) if already at the goal (within 1 mm).
    """
    return normalize(goal_x - my_x, goal_y - my_y)


# ---------------------------------------------------------------------------
# Velocity conversion
# ---------------------------------------------------------------------------

def force_to_cmd_vel(
    fx: float,
    fy: float,
    robot_theta: float,
    max_linear: float,
    max_angular: float,
    angular_gain: float = 2.5,
) -> Tuple[float, float]:
    """
    Convert a world-frame force vector (fx, fy) into differential-drive
    commands (linear_x, angular_z).

    Strategy:
      1. Compute desired heading from atan2(fy, fx).
      2. Compute heading error relative to current orientation.
      3. angular_z is proportional to heading_error (P-controller).
      4. linear_x is proportional to force magnitude and the cosine of the
         heading error so the robot slows when turning sharply.

    Returns (linear_x, angular_z) already clamped to [−max, +max].
    """
    force_mag = math.hypot(fx, fy)
    if force_mag < 1e-6:
        # No net force — stop smoothly
        return (0.0, 0.0)

    desired_heading = math.atan2(fy, fx)
    heading_error = normalize_angle(desired_heading - robot_theta)

    angular_z = clamp(angular_gain * heading_error, -max_angular, max_angular)

    # Cosine factor: 1.0 when perfectly aligned, 0.0 when heading error > 90°
    alignment_factor = max(0.0, math.cos(heading_error))
    linear_x = clamp(force_mag * alignment_factor * max_linear, 0.0, max_linear)

    return (linear_x, angular_z)
