# PROJECT A: Flocking with Obstacle Navigation

## 1. Package Structure

```
swarm_flocking_ws/
└── src/
    ├── swarm_flocking/                    # Main Python package
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/
    │   │   └── swarm_flocking
    │   ├── swarm_flocking/
    │   │   ├── __init__.py
    │   │   ├── boid_node.py              # Core: one instance per robot
    │   │   ├── flock_monitor_node.py     # Metrics + centroid publisher
    │   │   └── utils/
    │   │       ├── __init__.py
    │   │       ├── reynolds.py           # Separation, alignment, cohesion math
    │   │       └── obstacle_avoidance.py # LaserScan → repulsive force
    │   ├── launch/
    │   │   ├── spawn_flock.launch.py     # Spawn N robots in Gazebo
    │   │   ├── flocking.launch.py        # Start all boid nodes (no Gazebo)
    │   │   └── full_sim.launch.py        # Gazebo + RViz + spawn + flocking
    │   ├── config/
    │   │   ├── flocking_params.yaml      # Tunable weights and thresholds
    │   │   └── rviz_config.rviz          # Pre-configured RViz layout
    │   └── test/
    │       ├── test_reynolds.py          # Unit tests for force calculations
    │       └── test_integration.py       # Full 3-robot smoke test
    │
    ├── swarm_flocking_gazebo/             # Gazebo world + models
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── worlds/
    │   │   ├── obstacle_course.world     # Main corridor world (30×15m)
    │   │   └── open_field.world          # Simple test world
    │   ├── models/
    │   │   ├── narrow_wall/              # Bottleneck wall segments
    │   │   │   ├── model.config
    │   │   │   └── model.sdf
    │   │   └── goal_marker/              # Visual goal zone
    │   │       ├── model.config
    │   │       └── model.sdf
    │   └── launch/
    │       └── gazebo_world.launch.py
    │
    └── swarm_interfaces/                  # Custom message definitions
        ├── package.xml
        ├── CMakeLists.txt
        └── msg/
            └── FlockState.msg            # Aggregated flock metrics
```

## 2. Configuration File — `flocking_params.yaml`

```yaml
# config/flocking_params.yaml
# Uses ROS 2 YAML parameter format with wildcard namespace

/**:
  ros__parameters:
    num_robots: 6

    # ─── Reynolds Force Weights ─────────────────────────────
    w_separation: 1.5     # Highest priority: avoid robot-robot collision
    w_alignment:  1.0     # Match flock's average heading
    w_cohesion:   1.0     # Pull toward flock centroid
    w_obstacle:   2.5     # Reactive obstacle avoidance (LaserScan-based)
    w_migration:  0.3     # Weak global pull toward current waypoint

    # ─── Sensing Ranges (meters) ────────────────────────────
    neighbor_radius:    3.0   # Max range to consider another robot a neighbor
    separation_radius:  0.8   # Below this, separation force activates
    obstacle_threshold: 0.6   # Laser reading below this triggers avoidance

    # ─── Motion Limits ──────────────────────────────────────
    max_linear_vel:  0.20     # m/s (TurtleBot3 Burger hardware limit: 0.22 m/s)
    max_angular_vel: 1.5      # rad/s

    # ─── Waypoints (flat list: x0, y0, x1, y1, ...) ────────
    waypoints:
      - 8.0    # WP0 x — approach bottleneck
      - 7.5    # WP0 y
      - 15.0   # WP1 x — cross open field
      - 7.5    # WP1 y
      - 22.0   # WP2 x — enter maze section
      - 7.5    # WP2 y
      - 28.0   # WP3 x — reach goal zone
      - 7.5    # WP3 y
```

> **Note**: Waypoints use a flat list format (`[x0, y0, x1, y1, ...]`) since ROS 2 parameters don't support nested lists. Parsed into `(x, y)` tuples in `boid_node.py`.

## 3. Custom Message — `FlockState.msg`

```
# swarm_interfaces/msg/FlockState.msg
std_msgs/Header header
geometry_msgs/Point centroid           # Flock center of mass
float32 cohesion_radius                # Average distance from centroid
float32 avg_speed                      # Mean linear velocity
int32 num_active_robots                # Robots currently flocking
int32 collision_count                  # Cumulative near-collision events detected
bool is_split                          # True if flock fragmented into subgroups
int32 num_subgroups                    # Number of connected components (added field)
```

> **Changed from plan**: Added `num_subgroups` field for more detailed split detection info.

## 4. Core Node — `boid_node.py`

```python
#!/usr/bin/env python3
"""
boid_node.py — One instance per robot.
Subscribes to own /odom (nav_msgs/Odometry), computes Reynolds forces +
obstacle avoidance, publishes cmd_vel.

Key design choices (changed from original pseudocode):
  - Subscribes to nav_msgs/Odometry (not PoseStamped) for odom.
  - Applies spawn_x/spawn_y offset to convert odom-local to world-frame.
  - Parameters declared individually (no bulk YAML API).
  - Lambda capture with rid=i default-arg (avoids closure bug).
  - Stale neighbour timeout: entries older than 2.0s are ignored.
  - Exponential low-pass filter on output velocity to suppress jitter.
  - Pure math delegated to utils.reynolds and utils.obstacle_avoidance.
  - Custom QoS profiles: BEST_EFFORT for sensors, RELIABLE for inter-robot.
"""

import math, time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)

from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from swarm_flocking.utils.reynolds import (
    yaw_from_quaternion, normalize_angle, clamp, normalize,
    compute_separation, compute_alignment, compute_cohesion,
    compute_migration, force_to_cmd_vel,
)
from swarm_flocking.utils.obstacle_avoidance import laser_to_repulsive_force

# Constants
STALE_TIMEOUT_S = 2.0
LPF_ALPHA = 0.4          # Low-pass filter coefficient (0 = max smooth, 1 = raw)
WAYPOINT_ARRIVAL_RADIUS = 1.2

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)
STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

class NeighbourPose:
    """Cached neighbour position with monotonic timestamp."""
    __slots__ = ('x', 'y', 'theta', 'timestamp')
    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta
        self.timestamp = time.monotonic()

class NeighbourVel:
    """Cached neighbour velocity with monotonic timestamp."""
    __slots__ = ('vx', 'vy', 'timestamp')
    def __init__(self, vx, vy):
        self.vx, self.vy = vx, vy
        self.timestamp = time.monotonic()


class BoidNode(Node):
    def __init__(self):
        super().__init__('boid_node')

        # --- Parameters (declared individually, not bulk YAML) ---
        self._declare_params()
        self._read_params()

        # --- State ---
        self.my_pose: Optional[Tuple[float,float,float]] = None  # (x, y, theta) world-frame
        self.my_vel: Tuple[float,float] = (0.0, 0.0)   # world-frame velocity
        self.latest_scan: Optional[LaserScan] = None
        self.neighbour_poses: Dict[int, NeighbourPose] = {}
        self.neighbour_vels:  Dict[int, NeighbourVel]  = {}
        self._smooth_lin: float = 0.0   # low-pass filter state
        self._smooth_ang: float = 0.0
        self.current_wp: int = 0

        # --- Publishers (absolute topic paths) ---
        ns = f'/robot_{self.robot_id}'
        self.cmd_pub  = self.create_publisher(Twist, f'{ns}/cmd_vel', STATE_QOS)
        self.pose_pub = self.create_publisher(PoseStamped, f'{ns}/pose_share', STATE_QOS)
        self.vel_pub  = self.create_publisher(TwistStamped, f'{ns}/velocity_share', STATE_QOS)

        # --- Subscribers: own sensors ---
        self.create_subscription(Odometry, f'{ns}/odom', self._odom_callback, SENSOR_QOS)
        self.create_subscription(LaserScan, f'{ns}/scan', self._scan_callback, SENSOR_QOS)

        # --- Subscribers: neighbour state (one per peer) ---
        for i in range(self.num_robots):
            if i == self.robot_id:
                continue
            self.create_subscription(
                PoseStamped, f'/robot_{i}/pose_share',
                lambda msg, rid=i: self._neighbour_pose_callback(rid, msg), STATE_QOS)
            self.create_subscription(
                TwistStamped, f'/robot_{i}/velocity_share',
                lambda msg, rid=i: self._neighbour_vel_callback(rid, msg), STATE_QOS)

        # --- Main loop at 10 Hz ---
        self.timer = self.create_timer(0.1, self._flocking_loop)

    # ── Parameter declaration ─────────────────────────────────────

    def _declare_params(self):
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_robots', 6)
        self.declare_parameter('w_separation', 1.5)
        self.declare_parameter('w_alignment', 1.0)
        self.declare_parameter('w_cohesion', 1.0)
        self.declare_parameter('w_obstacle', 2.5)
        self.declare_parameter('w_migration', 0.3)
        self.declare_parameter('neighbor_radius', 3.0)
        self.declare_parameter('separation_radius', 0.8)
        self.declare_parameter('obstacle_threshold', 0.6)
        self.declare_parameter('max_linear_vel', 0.20)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('waypoints', [12.0, 1.0, 12.0, 7.0, 12.0, 13.0])

    def _read_params(self):
        self.robot_id   = int(self.get_parameter('robot_id').value)
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.w_sep  = float(self.get_parameter('w_separation').value)
        self.w_ali  = float(self.get_parameter('w_alignment').value)
        self.w_coh  = float(self.get_parameter('w_cohesion').value)
        self.w_obs  = float(self.get_parameter('w_obstacle').value)
        self.w_mig  = float(self.get_parameter('w_migration').value)
        self.neighbour_r = float(self.get_parameter('neighbor_radius').value)
        self.sep_r       = float(self.get_parameter('separation_radius').value)
        self.obs_thresh  = float(self.get_parameter('obstacle_threshold').value)
        self.max_lin     = float(self.get_parameter('max_linear_vel').value)
        self.max_ang     = float(self.get_parameter('max_angular_vel').value)
        self.spawn_x     = float(self.get_parameter('spawn_x').value)
        self.spawn_y     = float(self.get_parameter('spawn_y').value)
        # Parse flat waypoint list into (x, y) tuples
        flat = list(self.get_parameter('waypoints').value)
        if len(flat) % 2 != 0:
            flat = flat[:-1]
        self.waypoints = [(flat[k], flat[k+1]) for k in range(0, len(flat), 2)]

    # ── Callbacks ─────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        """Extract pose from /odom (nav_msgs/Odometry).
        CRITICAL: Gazebo's diff_drive odom starts at (0,0) per robot.
        We add (spawn_x, spawn_y) to convert to world-frame.
        """
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        theta = yaw_from_quaternion(ori)
        world_x = pos.x + self.spawn_x
        world_y = pos.y + self.spawn_y
        self.my_pose = (world_x, world_y, theta)

        # Body-frame velocity → world-frame
        vx_body = msg.twist.twist.linear.x
        vy_body = msg.twist.twist.linear.y
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        self.my_vel = (vx_body*cos_t - vy_body*sin_t,
                       vx_body*sin_t + vy_body*cos_t)

        # Broadcast world-frame state immediately
        self._publish_own_state(world_x, world_y, theta, ori)

    def _scan_callback(self, msg):
        self.latest_scan = msg

    def _neighbour_pose_callback(self, robot_id, msg):
        theta = yaw_from_quaternion(msg.pose.orientation)
        self.neighbour_poses[robot_id] = NeighbourPose(
            msg.pose.position.x, msg.pose.position.y, theta)

    def _neighbour_vel_callback(self, robot_id, msg):
        self.neighbour_vels[robot_id] = NeighbourVel(
            msg.twist.linear.x, msg.twist.linear.y)

    # ── State broadcasting ────────────────────────────────────────

    def _publish_own_state(self, x, y, theta, orientation):
        stamp = self.get_clock().now().to_msg()
        # PoseStamped (world-frame)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation = orientation
        self.pose_pub.publish(pose_msg)
        # TwistStamped (world-frame velocity)
        vel_msg = TwistStamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = 'map'
        vel_msg.twist.linear.x = self.my_vel[0]
        vel_msg.twist.linear.y = self.my_vel[1]
        self.vel_pub.publish(vel_msg)

    # ── Main Flocking Loop (10 Hz) ────────────────────────────────

    def _flocking_loop(self):
        if self.my_pose is None:
            return
        my_x, my_y, my_theta = self.my_pose
        my_vx, my_vy = self.my_vel

        # Step 1: Collect valid, non-stale neighbours
        neighbours = self._get_valid_neighbours(my_x, my_y)

        # Step 2: Compute each force (all from utils modules)
        f_sep = compute_separation(my_x, my_y, neighbours, self.sep_r)
        f_ali = compute_alignment(my_vx, my_vy, neighbours)
        f_coh = compute_cohesion(my_x, my_y, neighbours)
        f_obs = laser_to_repulsive_force(self.latest_scan, my_theta, self.obs_thresh)
        f_mig = self._get_migration_force(my_x, my_y)

        # Step 3: Weighted sum
        fx = (self.w_sep*f_sep[0] + self.w_ali*f_ali[0] + self.w_coh*f_coh[0]
              + self.w_obs*f_obs[0] + self.w_mig*f_mig[0])
        fy = (self.w_sep*f_sep[1] + self.w_ali*f_ali[1] + self.w_coh*f_coh[1]
              + self.w_obs*f_obs[1] + self.w_mig*f_mig[1])

        # Step 4: Convert force → (linear, angular) commands
        lin, ang = force_to_cmd_vel(fx, fy, my_theta, self.max_lin, self.max_ang)

        # Step 5: Exponential low-pass filter
        self._smooth_lin = LPF_ALPHA * lin + (1.0-LPF_ALPHA) * self._smooth_lin
        self._smooth_ang = LPF_ALPHA * ang + (1.0-LPF_ALPHA) * self._smooth_ang

        # Step 6: Clamp and publish
        cmd = Twist()
        cmd.linear.x  = clamp(self._smooth_lin, -self.max_lin, self.max_lin)
        cmd.angular.z = clamp(self._smooth_ang, -self.max_ang, self.max_ang)
        self.cmd_pub.publish(cmd)

        # Step 7: Waypoint advancement
        self._advance_waypoint(my_x, my_y)

    # ── Neighbour helper ──────────────────────────────────────────

    def _get_valid_neighbours(self, my_x, my_y):
        """Return list of (id, x, y, vx, vy, dist) for non-stale, in-range robots."""
        now = time.monotonic()
        result = []
        for rid, np in self.neighbour_poses.items():
            if (now - np.timestamp) > STALE_TIMEOUT_S:
                continue
            dist = math.hypot(np.x - my_x, np.y - my_y)
            if dist > self.neighbour_r:
                continue
            nv = self.neighbour_vels.get(rid)
            vx = vy = 0.0
            if nv and (now - nv.timestamp) <= STALE_TIMEOUT_S:
                vx, vy = nv.vx, nv.vy
            result.append((rid, np.x, np.y, vx, vy, dist))
        return result

    # ── Migration force ───────────────────────────────────────────

    def _get_migration_force(self, my_x, my_y):
        if self.current_wp >= len(self.waypoints):
            return (0.0, 0.0)
        gx, gy = self.waypoints[self.current_wp]
        return compute_migration(my_x, my_y, gx, gy)

    def _advance_waypoint(self, my_x, my_y):
        if self.current_wp >= len(self.waypoints):
            return
        gx, gy = self.waypoints[self.current_wp]
        if math.hypot(gx-my_x, gy-my_y) < WAYPOINT_ARRIVAL_RADIUS:
            self.get_logger().info(
                f'robot_{self.robot_id} reached waypoint {self.current_wp}')
            self.current_wp += 1


def main(args=None):
    rclpy.init(args=args)
    node = BoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Key differences from original pseudocode:
- Subscribes to `nav_msgs/Odometry` (not `PoseStamped`) for own position
- Adds `spawn_x`/`spawn_y` offset to convert Gazebo odom-local to world-frame coordinates
- Parameters declared individually via `declare_parameter()`, not the non-existent `declare_parameters_from_yaml()`
- Custom QoS profiles (BEST_EFFORT for sensors, RELIABLE for inter-robot state)
- Stale neighbour timeout (2.0s) — old data is ignored
- Exponential low-pass filter (α=0.4) on command output to suppress oscillation
- All Reynolds math delegated to `utils/reynolds.py` module functions
- Obstacle avoidance delegated to `utils/obstacle_avoidance.py`
- `NeighbourPose` / `NeighbourVel` data classes with monotonic timestamps

## 5. Reynolds Math Utilities — `utils/reynolds.py`

```python
"""
reynolds.py — Pure, stateless math functions for the three classic boid
rules plus helper geometry functions.

All forces return normalised unit 2-D vectors (fx, fy).

Neighbour tuple format: (robot_id, nx, ny, vx, vy, dist)
"""
import math
from typing import List, Tuple

Vec2 = Tuple[float, float]
NeighbourEntry = Tuple[int, float, float, float, float, float]

def normalize(x, y, epsilon=1e-6) -> Vec2:
    mag = math.hypot(x, y)
    return (0.0, 0.0) if mag < epsilon else (x/mag, y/mag)

def clamp(value, min_val, max_val) -> float:
    return max(min_val, min(max_val, value))

def normalize_angle(angle) -> float:
    """Wrap angle to [-π, π] using modulo (handles large values)."""
    angle = math.fmod(angle + math.pi, 2.0 * math.pi)
    if angle < 0.0:
        angle += 2.0 * math.pi
    return angle - math.pi

def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def compute_separation(my_x, my_y, neighbours, separation_radius) -> Vec2:
    """Inverse-square repulsion from neighbours within separation_radius."""
    fx = fy = 0.0; count = 0
    for (_rid, nx, ny, _vx, _vy, dist) in neighbours:
        if dist < separation_radius and dist > 1e-6:
            weight = 1.0 / (dist * dist)
            fx += (my_x - nx) * weight
            fy += (my_y - ny) * weight
            count += 1
    if count == 0: return (0.0, 0.0)
    return normalize(fx/count, fy/count)

def compute_alignment(my_vx, my_vy, neighbours) -> Vec2:
    """Steer to match average neighbour velocity."""
    if not neighbours: return (0.0, 0.0)
    n = len(neighbours)
    avg_vx = sum(vx for (_, _, _, vx, _, _) in neighbours) / n
    avg_vy = sum(vy for (_, _, _, _, vy, _) in neighbours) / n
    return normalize(avg_vx - my_vx, avg_vy - my_vy)

def compute_cohesion(my_x, my_y, neighbours) -> Vec2:
    """Steer toward centroid of neighbours."""
    if not neighbours: return (0.0, 0.0)
    n = len(neighbours)
    cx = sum(nx for (_, nx, _, _, _, _) in neighbours) / n
    cy = sum(ny for (_, _, ny, _, _, _) in neighbours) / n
    return normalize(cx - my_x, cy - my_y)

def compute_migration(my_x, my_y, goal_x, goal_y) -> Vec2:
    """Normalised direction toward waypoint."""
    return normalize(goal_x - my_x, goal_y - my_y)

def force_to_cmd_vel(fx, fy, robot_theta, max_linear, max_angular,
                     angular_gain=2.5) -> Tuple[float, float]:
    """
    World-frame force → differential-drive commands.
    angular_gain = 2.5 (P-controller gain, changed from original 2.0).
    linear_x clamped to [0, max_linear] (never negative).
    """
    force_mag = math.hypot(fx, fy)
    if force_mag < 1e-6:
        return (0.0, 0.0)
    desired_heading = math.atan2(fy, fx)
    heading_error = normalize_angle(desired_heading - robot_theta)
    angular_z = clamp(angular_gain * heading_error, -max_angular, max_angular)
    alignment_factor = max(0.0, math.cos(heading_error))
    linear_x = clamp(force_mag * alignment_factor * max_linear, 0.0, max_linear)
    return (linear_x, angular_z)
```

### Key differences from original pseudocode:
- All functions are standalone (not class methods) — pure math module
- `force_to_cmd_vel` uses `angular_gain=2.5` (was 2.0 in pseudocode)
- `force_to_cmd_vel` clamps linear to `[0, max]` (never negative — robots can't reverse)
- `normalize_angle` uses `math.fmod` (was while-loop in pseudocode)
- Each function takes explicit parameters instead of reading from `self`

## 6. Obstacle Avoidance — `utils/obstacle_avoidance.py`

```python
"""
obstacle_avoidance.py — Converts LaserScan into a world-frame repulsive force.

Improvements over inline pseudocode:
  - Ray stride to limit CPU on high-resolution scans (max 360 rays processed)
  - Soft proximity ramp: force fades near obstacle_threshold, no sudden jumps
  - Safe distance clamping to avoid blow-up near range_min
  - Averages across contributing rays instead of raw accumulation
  - Returns (0, 0) for None scan or no obstacles
"""
import math

MAX_RAYS_PROCESSED = 360

def laser_to_repulsive_force(scan, robot_theta, obstacle_threshold):
    if scan is None:
        return (0.0, 0.0)

    fx_robot = fy_robot = 0.0
    hit_count = 0
    total_rays = len(scan.ranges)
    stride = max(1, total_rays // MAX_RAYS_PROCESSED)

    for i in range(0, total_rays, stride):
        dist = scan.ranges[i]
        if not math.isfinite(dist):
            continue
        if dist < scan.range_min or dist >= obstacle_threshold:
            continue

        angle = scan.angle_min + i * scan.angle_increment
        obs_x = dist * math.cos(angle)
        obs_y = dist * math.sin(angle)

        proximity_factor = (obstacle_threshold - dist) / obstacle_threshold
        safe_dist = max(dist, scan.range_min + 1e-3)
        weight = proximity_factor / (safe_dist * safe_dist)

        fx_robot -= obs_x * weight
        fy_robot -= obs_y * weight
        hit_count += 1

    if hit_count == 0:
        return (0.0, 0.0)

    fx_robot /= hit_count
    fy_robot /= hit_count

    # Robot frame → world frame
    cos_t, sin_t = math.cos(robot_theta), math.sin(robot_theta)
    fx_world = fx_robot * cos_t - fy_robot * sin_t
    fy_world = fx_robot * sin_t + fy_robot * cos_t

    mag = math.hypot(fx_world, fy_world)
    if mag < 1e-6:
        return (0.0, 0.0)
    return (fx_world / mag, fy_world / mag)
```

## 7. Flock Monitor Node — `flock_monitor_node.py`

```python
"""
flock_monitor_node.py — Fully implemented singleton node.

Publishes at 2 Hz:
  - /flock/centroid       (PointStamped)
  - /flock/convex_hull    (Marker — LINE_STRIP, green)
  - /flock/neighbor_links (MarkerArray — thin blue lines)
  - /flock/state          (FlockState)

Implemented features:
  1. Centroid computation
  2. Cohesion radius (mean distance from centroid)
  3. Average speed across all active robots
  4. Near-collision detection (< 0.25m) with cumulative counter
  5. BFS-based graph split detection (connected components)
  6. Graham scan convex hull visualization
  7. Neighbour link visualization
"""

class FlockMonitorNode(Node):
    def __init__(self):
        super().__init__('flock_monitor')
        self.num_robots = ...  # from params
        self.robot_states = {i: RobotState() for i in range(self.num_robots)}
        self._cumulative_collisions = 0
        self._prev_collision_pairs = set()

        # Subscribe to all robots' pose_share and velocity_share
        for i in range(self.num_robots):
            self.create_subscription(PoseStamped, f'/robot_{i}/pose_share', ...)
            self.create_subscription(TwistStamped, f'/robot_{i}/velocity_share', ...)

        # Publishers
        self.centroid_pub = self.create_publisher(PointStamped, '/flock/centroid', 10)
        self.hull_pub     = self.create_publisher(Marker, '/flock/convex_hull', 10)
        self.links_pub    = self.create_publisher(MarkerArray, '/flock/neighbor_links', 10)
        self.state_pub    = self.create_publisher(FlockState, '/flock/state', 10)

        self.timer = self.create_timer(0.5, self._compute_and_publish)  # 2 Hz

    def _compute_and_publish(self):
        """
        Full implementation (not pseudocode):
        1. Filter active robots (pose_ts not stale)
        2. Centroid = mean(positions)
        3. Cohesion radius = mean(distance from centroid)
        4. Avg speed = mean(hypot(vx, vy))
        5. Collision detection: pairs < 0.25m, track new events cumulatively
        6. Split detection via BFS connected components
        7. Publish FlockState with all metrics
        8. Publish centroid PointStamped
        9. Publish convex hull LINE_STRIP marker (Graham scan)
        10. Publish neighbour link MarkerArray
        """
        # ... fully implemented — see source code ...

    def _detect_split(self, active_ids):
        """BFS adjacency graph → (num_components, is_split)."""
        # Build adjacency: edge if dist <= neighbour_radius
        # BFS to count connected components
        # Returns (num_subgroups, num_subgroups > 1)

    def _convex_hull(self, points):
        """Graham scan convex hull — handles degenerate cases."""

    def _build_hull_marker(self, active_ids, stamp):
        """LINE_STRIP marker tracing the convex hull."""

    def _build_link_markers(self, active_ids, stamp):
        """MarkerArray of thin LINE_LIST markers for neighbour pairs."""
```

### Key differences from original pseudocode:
- **Fully implemented** — not just pseudocode anymore
- Collision detection with `COLLISION_RADIUS = 0.25m` and cumulative tracking
- BFS-based split detection with `num_subgroups` count
- Full Graham scan convex hull with degenerate case handling
- Proper marker deletion/recreation for link markers (avoids ghost lines)
- Stale data filtering on robot poses

## 8. Launch File — `full_sim.launch.py`

```python
"""
full_sim.launch.py — Master launch file.

Starts (in order):
  1. Gazebo with obstacle_course.world
  2. Robot State Publishers for TF (URDF — no Gazebo plugins) — at ~28s
  3. Entity Spawns from SDF (HAS Gazebo plugins) — at ~30s, staggered 1.5s apart
  4. Boid Nodes — at ~35s, staggered
  5. Flock Monitor (singleton)
  6. RViz with pre-configured layout

Key design choices:
  - TimerAction-based staggered delays (not simultaneous)
  - 30s Gazebo ready delay to prevent race conditions
  - Robot spawned from SDF model (has diff_drive, lidar, IMU plugins)
  - Robot State Publisher uses URDF (TF frames only, no plugins)
  - spawn_x/spawn_y passed to each boid_node for odom→world conversion
  - FastDDS (rmw_fastrtps_cpp) required — CycloneDDS hits participant limits
"""
```

### Key differences from original pseudocode:
- Uses `TimerAction` with staggered delays (not simultaneous spawn)
- Spawns from SDF model file (not `-topic robot_description`)
- Separate robot_state_publisher for TF (using URDF)
- Includes `GAZEBO_MODEL_PATH` environment variable setup
- Passes `spawn_x`, `spawn_y` as parameters to each boid_node
- Includes an RViz node with config file
- `flocking.launch.py` added: starts only boid nodes (no Gazebo spawn)

## 9. Gazebo World — `obstacle_course.world`

```xml
<!-- 30×15m structured obstacle course -->
<!-- Layout:
     ┌──────────────────────────────────────────────────────────────┐ Y=15
     │  START    │■ BOTTLENECK ■│  OPEN FIELD  │  MAZE  │  GOAL   │
     │  (2,4.5)  │  gap ≈ 1.5m  │   divider    │ 5 walls│ (28,7.5)│
     └──────────────────────────────────────────────────────────────┘ Y=0
     X: 0────────8.5────────14──────────20────────27──────30
-->
```

### Key differences from original pseudocode:
- **All 4 boundary walls** implemented (north, south, east, west)
- Bottleneck walls at X=8.5 (was X=8), 1.5m gap centered at Y=7.5
- Added `divider_south` partial wall at X=14
- **5 maze walls** implemented (was just "more maze walls" comment)
- Color-coded walls: grey boundaries, orange bottleneck, purple maze
- Physics configured (ODE solver, 1000 Hz update rate)
- Scene ambient and shadow settings

## 10. Unit Tests — `test/test_reynolds.py`

Comprehensive unit tests covering all Reynolds math functions:

| Test Group | Tests |
|-----------|-------|
| `normalize()` | zero, near-zero, unit-x, unit-y, diagonal, magnitude, negatives |
| `clamp()` | within range, below min, above max, at boundaries, negative range |
| `normalize_angle()` | zero, π wrapping, 2π, large values |
| `compute_separation()` | no neighbours, outside zone, inside zone pushes away, symmetric cancel, unit vector result |
| `compute_alignment()` | empty, aligned (zero correction), different direction |
| `compute_cohesion()` | empty, centroid at origin, pull direction, symmetric cancel |
| `compute_migration()` | at goal (zero), direction, unit vector |
| `force_to_cmd_vel()` | zero force, aligned max speed, perpendicular slows, angular clamping, linear never negative |

## 11. Timeline

| Week | Tasks |
|------|-------|
| **1** | Set up workspace. Spawn multi-robot in Gazebo. Verify namespaced topics (`/robot_0/cmd_vel`, etc.). Build Gazebo world. |
| **2** | Implement `reynolds.py` (separation + alignment + cohesion). Test in open field. Tune weights. |
| **3** | Add obstacle avoidance from LaserScan. Add migration force. Test in obstacle_course world. Implement flock_monitor for RViz. |
| **4** | Parameter tuning. Record rosbag data. Generate evaluation plots. Document results. |

## 12. Key Commands to Run

```bash
# Terminal 1 — Launch everything
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/swarm_flocking_ws/swarm_flocking_ws/install/setup.bash
ros2 launch swarm_flocking full_sim.launch.py num_robots:=6

# Terminal 2 — Monitor flock state
ros2 topic echo /flock/state

# Terminal 3 — Live-tune a weight
ros2 param set /robot_0/boid_0 w_separation 2.0

# Terminal 4 — Record data for analysis
ros2 bag record /flock/state /flock/centroid -o flocking_experiment_1
```

---
---