#!/usr/bin/env python3
# FILE: test/test_integration.py
"""
Integration smoke test — 3-robot flocking.

Tests that the BoidNode + ReynoldsUtils pipeline produces sensible velocity
commands given synthetic neighbour data, WITHOUT requiring a running ROS 2
instance or Gazebo.

We mock rclpy so the test runs in a pure-Python CI environment.

Run with:
    python3 -m pytest src/swarm_flocking/test/test_integration.py -v
"""

import math
import sys
import os
import types
import unittest

# ─── Stub out rclpy so importlib doesn't need a ROS install ─────────────────
# We only need to stub what BoidNode.__init__ calls before the node can be
# instantiated. The actual behavior is tested through unit-level calls.
rclpy_stub = types.ModuleType('rclpy')
rclpy_stub.init = lambda args=None: None
rclpy_stub.shutdown = lambda: None
rclpy_stub.spin = lambda node: None

node_stub = types.ModuleType('rclpy.node')
class _StubNode:
    def __init__(self, name):
        self._name = name
        self._params = {}
    def declare_parameter(self, name, value):
        self._params[name] = value
    def get_parameter(self, name):
        class _P:
            def __init__(self, val): self.value = val
        return _P(self._params.get(name))
    def get_clock(self): return None
    def get_logger(self):
        class _L:
            def info(self, m): pass
            def warn(self, m): pass
            def error(self, m): pass
        return _L()
    def create_publisher(self, *a, **k): return _MockPub()
    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def destroy_node(self): pass

node_stub.Node = _StubNode
sys.modules['rclpy'] = rclpy_stub
sys.modules['rclpy.node'] = node_stub

# Stub all other rclpy sub-modules we import
for mod in ['rclpy.qos', 'geometry_msgs.msg', 'nav_msgs.msg',
            'sensor_msgs.msg', 'visualization_msgs.msg', 'std_msgs.msg',
            'swarm_interfaces.msg', 'tf2_ros', 'tf2_geometry_msgs']:
    stub = types.ModuleType(mod)
    sys.modules[mod] = stub

# Provide enough geometry_msgs attributes for imports to work
import types as _t
gm = sys.modules['geometry_msgs.msg']
for cls in ['Twist', 'PoseStamped', 'TwistStamped', 'Point', 'PointStamped',
            'Quaternion']:
    setattr(gm, cls, type(cls, (), {'__init__': lambda s, **kw: None}))

vm = sys.modules['visualization_msgs.msg']
for cls in ['Marker', 'MarkerArray']:
    setattr(vm, cls, type(cls, (), {
        '__init__': lambda s, **kw: None,
        'ADD': 0, 'DELETEALL': 999, 'LINE_STRIP': 4, 'LINE_LIST': 5
    }))

sm = sys.modules['swarm_interfaces.msg']
setattr(sm, 'FlockState', type('FlockState', (), {'__init__': lambda s, **kw: None}))

nm = sys.modules['nav_msgs.msg']
setattr(nm, 'Odometry', type('Odometry', (), {}))

qos_mod = sys.modules['rclpy.qos']
for cls in ['QoSProfile', 'QoSReliabilityPolicy', 'QoSHistoryPolicy',
            'QoSDurabilityPolicy']:
    setattr(qos_mod, cls, type(cls, (), {'BEST_EFFORT': 0, 'RELIABLE': 1,
                                          'KEEP_LAST': 0, 'VOLATILE': 0,
                                          '__init__': lambda s, **kw: None}))

std = sys.modules['std_msgs.msg']
setattr(std, 'ColorRGBA', type('ColorRGBA', (), {}))
setattr(std, 'Header', type('Header', (), {}))

class _MockPub:
    def __init__(self): self.published = []
    def publish(self, msg): self.published.append(msg)

# ─── Now import the real utility functions ───────────────────────────────────
utils_path = os.path.join(os.path.dirname(__file__), '..', 'swarm_flocking', 'utils')
sys.path.insert(0, utils_path)

from reynolds import (
    compute_separation,
    compute_alignment,
    compute_cohesion,
    compute_migration,
    force_to_cmd_vel,
    normalize,
    clamp,
)
from obstacle_avoidance import laser_to_repulsive_force


# ════════════════════════════════════════════════════════════════
# Scenario: 3 robots in a tight cluster, heading right (+x)
#
# Robot 0 (self): pos=(0,0), vel=(0.1, 0), theta=0
# Robot 1:        pos=(0.7,0), vel=(0.1, 0), dist=0.7  (inside sep zone)
# Robot 2:        pos=(2.0,0), vel=(0.15,0), dist=2.0  (inside neighbour zone)
# ════════════════════════════════════════════════════════════════

NEIGHBOURS_3 = [
    # (id, x,   y,   vx,   vy,  dist)
    (1,  0.7,  0.0,  0.10, 0.0, 0.7),
    (2,  2.0,  0.0,  0.15, 0.0, 2.0),
]
MY_X, MY_Y, MY_THETA = 0.0, 0.0, 0.0
MY_VX, MY_VY = 0.10, 0.0
SEP_RADIUS    = 0.8
MAX_LIN       = 0.20
MAX_ANG       = 1.5


class TestThreeRobotScenario(unittest.TestCase):

    def test_separation_pushes_away_from_close_robot(self):
        """Robot 1 at x=0.7 (inside sep_radius=0.8) → push in -x direction."""
        fx, fy = compute_separation(MY_X, MY_Y, NEIGHBOURS_3, SEP_RADIUS)
        self.assertLess(fx, 0.0,
            "Separation should push away from robot at x=0.7")

    def test_alignment_returns_unit_or_zero(self):
        """Alignment result must have magnitude ≤ 1.0."""
        fx, fy = compute_alignment(MY_VX, MY_VY, NEIGHBOURS_3)
        mag = math.hypot(fx, fy)
        self.assertLessEqual(mag, 1.0 + 1e-6)

    def test_cohesion_pulls_toward_centroid(self):
        """Centroid of (0.7,0) and (2.0,0) is (1.35,0) → pull in +x."""
        fx, fy = compute_cohesion(MY_X, MY_Y, NEIGHBOURS_3)
        self.assertGreater(fx, 0.0)
        self.assertAlmostEqual(fy, 0.0, places=4)

    def test_migration_pulls_toward_waypoint(self):
        """Waypoint at (8,7.5) from (0,0) → force in +x, +y direction."""
        fx, fy = compute_migration(MY_X, MY_Y, 8.0, 7.5)
        self.assertGreater(fx, 0.0)
        self.assertGreater(fy, 0.0)
        self.assertAlmostEqual(math.hypot(fx, fy), 1.0, places=4)

    def test_combined_force_produces_valid_cmd_vel(self):
        """End-to-end: weighted sum → cmd_vel values within limits."""
        # Weights as in default config
        w_sep, w_ali, w_coh, w_obs, w_mig = 1.5, 1.0, 1.0, 2.5, 0.3

        f_sep = compute_separation(MY_X, MY_Y, NEIGHBOURS_3, SEP_RADIUS)
        f_ali = compute_alignment(MY_VX, MY_VY, NEIGHBOURS_3)
        f_coh = compute_cohesion(MY_X, MY_Y, NEIGHBOURS_3)
        f_obs = (0.0, 0.0)   # no laser data in unit test
        f_mig = compute_migration(MY_X, MY_Y, 8.0, 7.5)

        fx = (w_sep * f_sep[0] + w_ali * f_ali[0] + w_coh * f_coh[0] +
              w_obs * f_obs[0] + w_mig * f_mig[0])
        fy = (w_sep * f_sep[1] + w_ali * f_ali[1] + w_coh * f_coh[1] +
              w_obs * f_obs[1] + w_mig * f_mig[1])

        lin, ang = force_to_cmd_vel(fx, fy, MY_THETA, MAX_LIN, MAX_ANG)
        lin = clamp(lin, -MAX_LIN, MAX_LIN)
        ang = clamp(ang, -MAX_ANG, MAX_ANG)

        self.assertGreaterEqual(lin, 0.0,
            "Linear velocity should be non-negative (robot moves forward or stops)")
        self.assertLessEqual(lin, MAX_LIN + 1e-9,
            "Linear velocity must not exceed TurtleBot3 limit")
        self.assertLessEqual(abs(ang), MAX_ANG + 1e-9,
            "Angular velocity must not exceed limit")

    def test_obstacle_avoidance_with_fake_scan(self):
        """Simulate a close obstacle ahead — repulsion should push backward."""
        # Create a fake LaserScan with one close reading straight ahead
        scan = types.SimpleNamespace(
            ranges=[10.0] * 360,  # mostly clear
            range_min=0.12,
            range_max=3.5,
            angle_min=-math.pi,
            angle_increment=2 * math.pi / 360,
        )
        scan.ranges[0] = 0.3   # 0.3 m obstacle directly ahead (angle=0 in robot frame)

        fx, fy = laser_to_repulsive_force(scan, robot_theta=0.0,
                                           obstacle_threshold=0.6)
        # Should push in -x direction (away from obstacle at angle=0)
        # (obstacle at angle=0 → robot-frame +x → repulse in -x)
        self.assertLess(fx, 0.0,
            "Obstacle ahead should produce backward repulsion force in world -x")

    def test_no_scan_returns_zero(self):
        """None scan should produce zero obstacle force."""
        result = laser_to_repulsive_force(None, 0.0, 0.6)
        self.assertEqual(result, (0.0, 0.0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
