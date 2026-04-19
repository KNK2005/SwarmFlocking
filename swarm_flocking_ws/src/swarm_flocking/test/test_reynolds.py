#!/usr/bin/env python3
# FILE: test/test_reynolds.py
"""
Unit tests for the Reynolds utility functions.

Tests:
  - normalize(): zero vector, unit vectors, arbitrary vectors
  - clamp(): boundary values, inside range, outside range
  - normalize_angle(): wrapping edge cases
  - compute_separation(): no neighbours, one neighbour inside zone,
                          multiple neighbours
  - compute_alignment(): empty, single, multiple
  - compute_cohesion(): empty, single centroid
  - compute_migration(): basic direction
  - force_to_cmd_vel(): aligned, 90° error, zero force

Run with:
    cd swarm_flocking_ws
    colcon build --packages-select swarm_flocking
    source install/setup.bash
    python3 -m pytest src/swarm_flocking/test/test_reynolds.py -v
"""

import math
import sys
import os

# Allow running without colcon install (direct path)
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'swarm_flocking', 'utils'))

from reynolds import (
    normalize,
    clamp,
    normalize_angle,
    compute_separation,
    compute_alignment,
    compute_cohesion,
    compute_migration,
    force_to_cmd_vel,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def approx_eq(a: float, b: float, tol: float = 1e-5) -> bool:
    return abs(a - b) < tol


def vec_approx(v1, v2, tol=1e-5):
    return approx_eq(v1[0], v2[0], tol) and approx_eq(v1[1], v2[1], tol)


# ---------------------------------------------------------------------------
# normalize()
# ---------------------------------------------------------------------------

class TestNormalize:
    def test_zero_vector(self):
        assert normalize(0.0, 0.0) == (0.0, 0.0)

    def test_near_zero_vector(self):
        assert normalize(1e-8, 1e-8) == (0.0, 0.0)

    def test_unit_x(self):
        fx, fy = normalize(5.0, 0.0)
        assert approx_eq(fx, 1.0) and approx_eq(fy, 0.0)

    def test_unit_y(self):
        fx, fy = normalize(0.0, -3.0)
        assert approx_eq(fx, 0.0) and approx_eq(fy, -1.0)

    def test_diagonal(self):
        fx, fy = normalize(1.0, 1.0)
        expected = 1.0 / math.sqrt(2)
        assert approx_eq(fx, expected) and approx_eq(fy, expected)

    def test_magnitude_is_one(self):
        fx, fy = normalize(3.0, 4.0)
        assert approx_eq(math.hypot(fx, fy), 1.0)

    def test_negative_components(self):
        fx, fy = normalize(-3.0, -4.0)
        assert approx_eq(math.hypot(fx, fy), 1.0)
        assert fx < 0 and fy < 0


# ---------------------------------------------------------------------------
# clamp()
# ---------------------------------------------------------------------------

class TestClamp:
    def test_within_range(self):
        assert clamp(0.5, 0.0, 1.0) == 0.5

    def test_below_min(self):
        assert clamp(-5.0, 0.0, 1.0) == 0.0

    def test_above_max(self):
        assert clamp(2.0, 0.0, 1.0) == 1.0

    def test_at_min(self):
        assert clamp(0.0, 0.0, 1.0) == 0.0

    def test_at_max(self):
        assert clamp(1.0, 0.0, 1.0) == 1.0

    def test_negative_range(self):
        assert clamp(-3.0, -5.0, -1.0) == -3.0


# ---------------------------------------------------------------------------
# normalize_angle()
# ---------------------------------------------------------------------------

class TestNormalizeAngle:
    def test_zero(self):
        assert approx_eq(normalize_angle(0.0), 0.0)

    def test_pi_wraps(self):
        # +pi should wrap to -pi (or stay at pi within ±π tolerance)
        result = normalize_angle(math.pi)
        assert abs(result) <= math.pi + 1e-9

    def test_two_pi(self):
        assert approx_eq(normalize_angle(2 * math.pi), 0.0)

    def test_negative_two_pi(self):
        assert approx_eq(normalize_angle(-2 * math.pi), 0.0)

    def test_three_half_pi(self):
        # 3π/2 → -π/2
        result = normalize_angle(3 * math.pi / 2)
        assert approx_eq(result, -math.pi / 2)

    def test_large_positive(self):
        result = normalize_angle(10 * math.pi)
        assert -math.pi <= result <= math.pi


# ---------------------------------------------------------------------------
# compute_separation()
# ---------------------------------------------------------------------------

class TestSeparation:
    # neighbour tuple: (id, x, y, vx, vy, dist)
    def test_no_neighbours(self):
        result = compute_separation(0.0, 0.0, [], separation_radius=0.8)
        assert result == (0.0, 0.0)

    def test_neighbour_outside_zone(self):
        # Neighbour at dist=1.5, zone=0.8 → no push
        neighbours = [(1, 1.5, 0.0, 0.0, 0.0, 1.5)]
        result = compute_separation(0.0, 0.0, neighbours, separation_radius=0.8)
        assert result == (0.0, 0.0)

    def test_neighbour_inside_zone_pushes_away(self):
        # Own pos (0,0), neighbour at (0.5, 0) → push in -x direction
        neighbours = [(1, 0.5, 0.0, 0.0, 0.0, 0.5)]
        fx, fy = compute_separation(0.0, 0.0, neighbours, separation_radius=0.8)
        assert fx < 0.0
        assert approx_eq(fy, 0.0, tol=1e-4)

    def test_two_neighbours_symmetric_cancel(self):
        # One neighbour on each side → forces should cancel to near-zero
        neighbours = [
            (1,  0.5, 0.0, 0.0, 0.0, 0.5),
            (2, -0.5, 0.0, 0.0, 0.0, 0.5),
        ]
        fx, fy = compute_separation(0.0, 0.0, neighbours, separation_radius=0.8)
        assert approx_eq(fx, 0.0, tol=1e-4)

    def test_result_is_unit_vector(self):
        neighbours = [(1, 0.3, 0.3, 0.0, 0.0, 0.3 * math.sqrt(2))]
        fx, fy = compute_separation(0.0, 0.0, neighbours, separation_radius=0.8)
        if fx != 0.0 or fy != 0.0:
            assert approx_eq(math.hypot(fx, fy), 1.0)


# ---------------------------------------------------------------------------
# compute_alignment()
# ---------------------------------------------------------------------------

class TestAlignment:
    def test_empty_returns_zero(self):
        assert compute_alignment(0.0, 0.0, []) == (0.0, 0.0)

    def test_single_neighbour_aligned(self):
        # Neighbour moves at (1,0), same as us → correction ≈ (0,0)
        neighbours = [(1, 2.0, 0.0, 1.0, 0.0, 2.0)]
        fx, fy = compute_alignment(1.0, 0.0, neighbours)
        # normalize(0,0) = (0,0)
        assert approx_eq(fx, 0.0) and approx_eq(fy, 0.0)

    def test_single_neighbour_different_direction(self):
        # Neighbour moves at (0,1), we move at (1,0) → push toward (0,1)
        neighbours = [(1, 2.0, 0.0, 0.0, 1.0, 2.0)]
        fx, fy = compute_alignment(1.0, 0.0, neighbours)
        # Correction = (0,1) - (1,0) = (-1,1), normalized
        assert fx < 0 and fy > 0

    def test_result_is_unit_or_zero(self):
        neighbours = [(1, 3.0, 0.0, 0.5, 0.3, 3.0)]
        fx, fy = compute_alignment(0.0, 0.0, neighbours)
        mag = math.hypot(fx, fy)
        assert approx_eq(mag, 1.0) or approx_eq(mag, 0.0)


# ---------------------------------------------------------------------------
# compute_cohesion()
# ---------------------------------------------------------------------------

class TestCohesion:
    def test_empty_returns_zero(self):
        assert compute_cohesion(0.0, 0.0, []) == (0.0, 0.0)

    def test_centroid_is_my_position(self):
        # Neighbours exactly at my position → zero force
        neighbours = [(1, 0.0, 0.0, 0.0, 0.0, 0.0)]
        result = compute_cohesion(0.0, 0.0, neighbours)
        # normalize(0,0) = (0,0)
        assert result == (0.0, 0.0)

    def test_pulls_toward_centroid(self):
        # Neighbour to my right → pull right
        neighbours = [(1, 5.0, 0.0, 0.0, 0.0, 5.0)]
        fx, fy = compute_cohesion(0.0, 0.0, neighbours)
        assert fx > 0 and approx_eq(fy, 0.0, tol=1e-4)

    def test_multiple_symmetric(self):
        # Two neighbours equidistant on both sides → centroid at origin
        neighbours = [
            (1,  3.0, 0.0, 0.0, 0.0, 3.0),
            (2, -3.0, 0.0, 0.0, 0.0, 3.0),
        ]
        fx, fy = compute_cohesion(0.0, 0.0, neighbours)
        assert approx_eq(fx, 0.0, tol=1e-4)

    def test_result_is_unit_vector(self):
        neighbours = [(1, 2.0, 3.0, 0.0, 0.0, math.hypot(2, 3))]
        fx, fy = compute_cohesion(0.0, 0.0, neighbours)
        assert approx_eq(math.hypot(fx, fy), 1.0)


# ---------------------------------------------------------------------------
# compute_migration()
# ---------------------------------------------------------------------------

class TestMigration:
    def test_at_goal_returns_zero(self):
        result = compute_migration(5.0, 5.0, 5.0, 5.0)
        assert result == (0.0, 0.0)

    def test_direction_toward_goal(self):
        fx, fy = compute_migration(0.0, 0.0, 1.0, 0.0)
        assert approx_eq(fx, 1.0) and approx_eq(fy, 0.0)

    def test_result_is_unit_vector(self):
        fx, fy = compute_migration(0.0, 0.0, 3.0, 4.0)
        assert approx_eq(math.hypot(fx, fy), 1.0)


# ---------------------------------------------------------------------------
# force_to_cmd_vel()
# ---------------------------------------------------------------------------

class TestForceToCmdVel:
    def test_zero_force_returns_zero(self):
        lin, ang = force_to_cmd_vel(0.0, 0.0, 0.0, 0.2, 1.5)
        assert approx_eq(lin, 0.0) and approx_eq(ang, 0.0)

    def test_aligned_force_max_speed(self):
        # Force pointing exactly in the direction we're facing
        lin, ang = force_to_cmd_vel(1.0, 0.0, 0.0, 0.2, 1.5)
        assert approx_eq(lin, 0.2)   # max linear
        assert approx_eq(ang, 0.0, tol=1e-3)

    def test_perpendicular_force_slows_down(self):
        # Force pointing 90° left (fy=1), facing forward (theta=0)
        lin, ang = force_to_cmd_vel(0.0, 1.0, 0.0, 0.2, 1.5)
        assert lin < 0.05   # slow when turning sharply
        assert ang > 0       # positive angular (turn left)

    def test_clamps_angular(self):
        # Very large heading error should be clamped
        lin, ang = force_to_cmd_vel(-1.0, 0.0, 0.0, 0.2, 1.5)
        assert abs(ang) <= 1.5 + 1e-6

    def test_linear_never_negative(self):
        for theta in [0.0, math.pi / 4, math.pi / 2, math.pi]:
            lin, _ = force_to_cmd_vel(1.0, 1.0, theta, 0.2, 1.5)
            assert lin >= 0.0


# ---------------------------------------------------------------------------
# Run standalone
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
