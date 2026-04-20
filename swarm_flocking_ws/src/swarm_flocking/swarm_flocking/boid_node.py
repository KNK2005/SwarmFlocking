#!/usr/bin/env python3
# FILE: swarm_flocking/boid_node.py
"""
BoidNode — One instance per robot.

Responsibilities:
  • Subscribe to own /odom (nav_msgs/Odometry) and publish a pose_share /
    velocity_share so sibling robots can read our state.
  • Subscribe to all peer robots' pose_share and velocity_share topics.
  • Compute the five Reynolds forces every 0.1 s (10 Hz).
  • Apply exponential low-pass smoothing to avoid jitter.
  • Publish cmd_vel respecting TurtleBot3 Burger velocity limits.

Key design choices:
  - Parameters loaded individually via declare_parameter / get_parameter
    (no non-existent YAML-bulk API).
  - Lambda capture with rid=i default arg (avoids classic closure bug).
  - Stale neighbour timeout: entries older than STALE_TIMEOUT_S are ignored.
  - Low-pass filter on the output velocity to suppress oscillation.
  - Pure math delegated to utils.reynolds and utils.obstacle_avoidance.
"""

import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from swarm_flocking.utils.reynolds import (
    yaw_from_quaternion,
    clamp,
    compute_separation,
    compute_alignment,
    compute_cohesion,
    compute_migration,
    force_to_cmd_vel,
)
from swarm_flocking.utils.obstacle_avoidance import laser_to_repulsive_force

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Seconds after which a neighbour's data is considered stale and ignored
STALE_TIMEOUT_S = 2.0

# Low-pass filter coefficient α ∈ (0, 1].  Smaller = smoother, more lag.
# 0.4 gives a good balance between responsiveness and smoothness at 10 Hz.
LPF_ALPHA = 0.4

# Waypoint arrival radius (meters)
WAYPOINT_ARRIVAL_RADIUS = 1.2

# QoS profile: best-effort for high-frequency sensor-like topics
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# QoS profile: reliable for inter-robot state sharing
STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)


# ---------------------------------------------------------------------------
# Helper dataclass-like named tuples (avoid external deps)
# ---------------------------------------------------------------------------

class NeighbourPose:
    __slots__ = ('x', 'y', 'theta', 'timestamp')

    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta
        self.timestamp = time.monotonic()


class NeighbourVel:
    __slots__ = ('vx', 'vy', 'timestamp')

    def __init__(self, vx: float, vy: float):
        self.vx = vx
        self.vy = vy
        self.timestamp = time.monotonic()


# ---------------------------------------------------------------------------
# BoidNode
# ---------------------------------------------------------------------------

class BoidNode(Node):
    """
    Core flocking node for a single robot.

    Parameters (set via launch file or command line):
        robot_id         : int   — unique integer ID of this robot
        num_robots       : int   — total number of robots in the swarm
        w_separation     : float — separation force weight
        w_alignment      : float — alignment force weight
        w_cohesion       : float — cohesion force weight
        w_obstacle       : float — obstacle avoidance weight
        w_migration      : float — migration (goal-seeking) weight
        neighbor_radius  : float — sensing radius (m)
        separation_radius: float — separation activation radius (m)
        obstacle_threshold: float— laser range triggering avoidance (m)
        max_linear_vel   : float — max cmd_vel linear.x (m/s)
        max_angular_vel  : float — max cmd_vel angular.z (rad/s)
        waypoint_{n}_x/y : float — sequential waypoint coordinates
    """

    def __init__(self):
        super().__init__('boid_node')

        # ----------------------------------------------------------------
        # Declare and read parameters
        # ----------------------------------------------------------------
        self._declare_params()
        self._read_params()

        # ----------------------------------------------------------------
        # State
        # ----------------------------------------------------------------
        self.my_pose: Optional[Tuple[float, float, float]] = None  # (x, y, theta)
        self.my_vel: Tuple[float, float] = (0.0, 0.0)  # (vx, vy) body frame → world
        self.latest_scan: Optional[LaserScan] = None

        # Keyed by int robot_id
        self.neighbour_poses: Dict[int, NeighbourPose] = {}
        self.neighbour_vels:  Dict[int, NeighbourVel]  = {}

        # Smoothed output velocities (low-pass filter state)
        self._smooth_lin: float = 0.0
        self._smooth_ang: float = 0.0

        # Waypoint pointer
        self.current_wp: int = 0

        # Cumulative collision counter (for FlockState)
        self.collision_count: int = 0

        # ----------------------------------------------------------------
        # Publishers — use absolute topic paths with robot namespace
        # ----------------------------------------------------------------
        ns = f'/robot_{self.robot_id}'

        self.cmd_pub = self.create_publisher(
            Twist, f'{ns}/cmd_vel', STATE_QOS)

        self.pose_pub = self.create_publisher(
            PoseStamped, f'{ns}/pose_share', STATE_QOS)

        self.vel_pub = self.create_publisher(
            TwistStamped, f'{ns}/velocity_share', STATE_QOS)

        # ----------------------------------------------------------------
        # Subscribers — own sensors (absolute paths)
        # ----------------------------------------------------------------
        self.create_subscription(
            Odometry, f'{ns}/odom',
            self._odom_callback, SENSOR_QOS)

        self.create_subscription(
            LaserScan, f'{ns}/scan',
            self._scan_callback, SENSOR_QOS)

        # ----------------------------------------------------------------
        # Subscribers — neighbour state (one per peer)
        # ----------------------------------------------------------------
        for i in range(self.num_robots):
            if i == self.robot_id:
                continue
            # Use default-argument capture (rid=i) to freeze the loop variable
            self.create_subscription(
                PoseStamped,
                f'/robot_{i}/pose_share',
                lambda msg, rid=i: self._neighbour_pose_callback(rid, msg),
                STATE_QOS,
            )
            self.create_subscription(
                TwistStamped,
                f'/robot_{i}/velocity_share',
                lambda msg, rid=i: self._neighbour_vel_callback(rid, msg),
                STATE_QOS,
            )

        # ----------------------------------------------------------------
        # Main control loop at 10 Hz
        # ----------------------------------------------------------------
        self.timer = self.create_timer(0.1, self._flocking_loop)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f'BoidNode started — robot_id={self.robot_id}, '
            f'num_robots={self.num_robots}, '
            f'spawn_offset=({self.spawn_x:.1f}, {self.spawn_y:.1f}), '
            f'odom_mode={"local+offset" if self.odom_is_local else "world"}, '
            f'waypoints={self.waypoints}')

    # ====================================================================
    # Parameter helpers
    # ====================================================================

    def _declare_params(self) -> None:
        """Declare all node parameters with sensible defaults."""
        self.declare_parameter('robot_id',          0)
        self.declare_parameter('num_robots',         6)
        self.declare_parameter('w_separation',       1.5)
        self.declare_parameter('w_alignment',        1.0)
        self.declare_parameter('w_cohesion',         1.0)
        self.declare_parameter('w_obstacle',         2.5)
        self.declare_parameter('w_migration',        0.3)
        self.declare_parameter('neighbor_radius',    3.0)
        self.declare_parameter('separation_radius',  0.8)
        self.declare_parameter('obstacle_threshold', 0.6)
        self.declare_parameter('max_linear_vel',     0.20)
        self.declare_parameter('max_angular_vel',    1.5)
        self.declare_parameter('lpf_alpha',          LPF_ALPHA)
        self.declare_parameter('waypoint_arrival_radius', WAYPOINT_ARRIVAL_RADIUS)
        self.declare_parameter('stop_on_goal_reached', True)
        
        # Adaptive scaling parameters
        self.declare_parameter('k_sep',              0.5)
        self.declare_parameter('min_sep_w',          1.0)
        self.declare_parameter('max_sep_w',          3.0)
        self.declare_parameter('alpha_coh',          0.2)
        self.declare_parameter('min_coh_w',          0.5)
        self.declare_parameter('max_coh_w',          2.0)
        self.declare_parameter('ideal_separation',   1.0)
        self.declare_parameter('threshold_spread',   1.5)
        self.declare_parameter('min_obs_w',          1.0)
        self.declare_parameter('max_obs_w',          5.0)
        self.declare_parameter('laser_epsilon',      0.01)

        # Spawn position offset: Gazebo's odom starts at (0,0) per robot.
        # We add these offsets to convert odom-frame pose to world-frame pose.
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        # If true, /odom starts at (0,0) in robot-local frame and needs
        # (spawn_x, spawn_y) offset. If false, /odom is already world-frame.
        self.declare_parameter('odom_is_local', True)
        # Waypoints stored as a flat list: [x0, y0, x1, y1, ...]
        self.declare_parameter('waypoints', [12.0, 1.0, 12.0, 7.0, 12.0, 13.0])

    def _read_params(self) -> None:
        """Read all declared parameters into instance attributes."""
        self.robot_id  = int(self.get_parameter('robot_id').value)
        self.num_robots = int(self.get_parameter('num_robots').value)
        self.w_sep     = float(self.get_parameter('w_separation').value)
        self.w_ali     = float(self.get_parameter('w_alignment').value)
        self.w_coh     = float(self.get_parameter('w_cohesion').value)
        self.w_obs     = float(self.get_parameter('w_obstacle').value)
        self.w_mig     = float(self.get_parameter('w_migration').value)
        self.neighbour_r  = float(self.get_parameter('neighbor_radius').value)
        self.sep_r        = float(self.get_parameter('separation_radius').value)
        self.obs_thresh   = float(self.get_parameter('obstacle_threshold').value)
        self.max_lin      = float(self.get_parameter('max_linear_vel').value)
        self.max_ang      = float(self.get_parameter('max_angular_vel').value)
        self.lpf_alpha    = float(self.get_parameter('lpf_alpha').value)
        self.wp_arrival_r = float(self.get_parameter('waypoint_arrival_radius').value)
        self.stop_on_goal = bool(self.get_parameter('stop_on_goal_reached').value)
        self.spawn_x      = float(self.get_parameter('spawn_x').value)
        self.spawn_y      = float(self.get_parameter('spawn_y').value)
        self.odom_is_local = bool(self.get_parameter('odom_is_local').value)

        # Adaptive scaling parameters
        self.k_sep         = float(self.get_parameter('k_sep').value)
        self.min_sep_w     = float(self.get_parameter('min_sep_w').value)
        self.max_sep_w     = float(self.get_parameter('max_sep_w').value)
        self.alpha_coh     = float(self.get_parameter('alpha_coh').value)
        self.min_coh_w     = float(self.get_parameter('min_coh_w').value)
        self.max_coh_w     = float(self.get_parameter('max_coh_w').value)
        self.ideal_sep     = float(self.get_parameter('ideal_separation').value)
        self.thresh_spread = float(self.get_parameter('threshold_spread').value)
        self.min_obs_w     = float(self.get_parameter('min_obs_w').value)
        self.max_obs_w     = float(self.get_parameter('max_obs_w').value)
        self.laser_eps     = float(self.get_parameter('laser_epsilon').value)

        # Parse flat waypoint list into list of (x, y) tuples
        flat = list(self.get_parameter('waypoints').value)
        if len(flat) % 2 != 0:
            self.get_logger().warn(
                'waypoints parameter has odd length; dropping last element.')
            flat = flat[:-1]
        self.waypoints: List[Tuple[float, float]] = [
            (flat[k], flat[k + 1]) for k in range(0, len(flat), 2)
        ]

        # Clamp runtime-tunable safety values after initial load.
        self.lpf_alpha = clamp(self.lpf_alpha, 0.05, 1.0)
        self.wp_arrival_r = max(0.1, self.wp_arrival_r)

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        """Handle runtime parameter updates for true live tuning."""
        try:
            for p in params:
                name = p.name
                value = p.value
                if name == 'w_separation':
                    self.w_sep = float(value)
                elif name == 'w_alignment':
                    self.w_ali = float(value)
                elif name == 'w_cohesion':
                    self.w_coh = float(value)
                elif name == 'w_obstacle':
                    self.w_obs = float(value)
                elif name == 'w_migration':
                    self.w_mig = float(value)
                elif name == 'neighbor_radius':
                    self.neighbour_r = max(0.1, float(value))
                elif name == 'separation_radius':
                    self.sep_r = max(0.05, float(value))
                elif name == 'obstacle_threshold':
                    self.obs_thresh = max(0.05, float(value))
                elif name == 'max_linear_vel':
                    self.max_lin = max(0.01, float(value))
                elif name == 'max_angular_vel':
                    self.max_ang = max(0.05, float(value))
                elif name == 'lpf_alpha':
                    self.lpf_alpha = clamp(float(value), 0.05, 1.0)
                elif name == 'waypoint_arrival_radius':
                    self.wp_arrival_r = max(0.1, float(value))
                elif name == 'stop_on_goal_reached':
                    self.stop_on_goal = bool(value)
                elif name == 'k_sep':
                    self.k_sep = float(value)
                elif name == 'min_sep_w':
                    self.min_sep_w = float(value)
                elif name == 'max_sep_w':
                    self.max_sep_w = float(value)
                elif name == 'alpha_coh':
                    self.alpha_coh = float(value)
                elif name == 'min_coh_w':
                    self.min_coh_w = float(value)
                elif name == 'max_coh_w':
                    self.max_coh_w = float(value)
                elif name == 'ideal_separation':
                    self.ideal_sep = max(0.05, float(value))
                elif name == 'threshold_spread':
                    self.thresh_spread = float(value)
                elif name == 'min_obs_w':
                    self.min_obs_w = float(value)
                elif name == 'max_obs_w':
                    self.max_obs_w = float(value)
                elif name == 'laser_epsilon':
                    self.laser_eps = max(1e-4, float(value))
                elif name == 'waypoints':
                    flat = [float(v) for v in list(value)]
                    if len(flat) % 2 != 0:
                        return SetParametersResult(
                            successful=False,
                            reason='waypoints must have even length [x0,y0,x1,y1,...]',
                        )
                    self.waypoints = [
                        (flat[k], flat[k + 1]) for k in range(0, len(flat), 2)
                    ]
                    if self.current_wp >= len(self.waypoints):
                        self.current_wp = max(0, len(self.waypoints) - 1)
                elif name == 'odom_is_local':
                    self.odom_is_local = bool(value)

            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    # ====================================================================
    # Callbacks
    # ====================================================================

    def _odom_callback(self, msg: Odometry) -> None:
        """Extract pose and velocity from /odom.

                Odom handling mode:
                    - odom_is_local=True: odom starts near (0,0) per robot and needs
                        (spawn_x, spawn_y) offset to recover world-frame coordinates.
                    - odom_is_local=False: odom is already world-referenced and should
                        be used as-is.
        """
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        theta = yaw_from_quaternion(ori)

        if self.odom_is_local:
            # Convert odom-local -> world-frame by adding spawn offset.
            world_x = pos.x + self.spawn_x
            world_y = pos.y + self.spawn_y
        else:
            # Odom is already world-referenced (typical with ros_gz Harmonic bridge).
            world_x = pos.x
            world_y = pos.y
        self.my_pose = (world_x, world_y, theta)

        # World-frame velocity (rotate body-frame twist by yaw)
        vx_body = msg.twist.twist.linear.x
        vy_body = msg.twist.twist.linear.y
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        self.my_vel = (
            vx_body * cos_t - vy_body * sin_t,
            vx_body * sin_t + vy_body * cos_t,
        )

        # Share world-frame state immediately after odom update
        self._publish_own_state(world_x, world_y, theta, ori)

    def _scan_callback(self, msg: LaserScan) -> None:
        """Cache the latest laser scan."""
        self.latest_scan = msg

    def _neighbour_pose_callback(self, robot_id: int, msg: PoseStamped) -> None:
        """Update neighbour pose cache."""
        theta = yaw_from_quaternion(msg.pose.orientation)
        self.neighbour_poses[robot_id] = NeighbourPose(
            msg.pose.position.x, msg.pose.position.y, theta)

    def _neighbour_vel_callback(self, robot_id: int, msg: TwistStamped) -> None:
        """Update neighbour velocity cache."""
        # TwistStamped.twist is a Twist (not geometry_msgs/TwistStamped sub-field)
        self.neighbour_vels[robot_id] = NeighbourVel(
            msg.twist.linear.x, msg.twist.linear.y)

    # ====================================================================
    # State broadcasting
    # ====================================================================

    def _publish_own_state(
        self, x: float, y: float, theta: float, orientation
    ) -> None:
        """Broadcast own pose and velocity to all other robots."""
        stamp = self.get_clock().now().to_msg()

        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = orientation
        self.pose_pub.publish(pose_msg)

        # TwistStamped — linear components are world-frame velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = 'map'
        vel_msg.twist.linear.x = self.my_vel[0]
        vel_msg.twist.linear.y = self.my_vel[1]
        self.vel_pub.publish(vel_msg)

    # ====================================================================
    # Main flocking loop (10 Hz)
    # ====================================================================

    def _flocking_loop(self) -> None:
        """Compute and publish a velocity command every 0.1 s."""
        if self.my_pose is None:
            # No odometry yet — hold still
            return

        # End-state behavior: once all waypoints are done, hold zero velocity.
        if self.stop_on_goal and self.current_wp >= len(self.waypoints):
            self._smooth_lin = 0.0
            self._smooth_ang = 0.0
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        my_x, my_y, my_theta = self.my_pose
        my_vx, my_vy = self.my_vel

        # Step 1: collect valid, non-stale neighbours
        neighbours = self._get_valid_neighbours(my_x, my_y)

        # Step 2: compute each force component (all return unit vectors)
        f_sep = compute_separation(my_x, my_y, neighbours, self.sep_r)
        f_ali = compute_alignment(my_vx, my_vy, neighbours)
        f_coh = compute_cohesion(my_x, my_y, neighbours)
        f_obs = laser_to_repulsive_force(
            self.latest_scan, my_theta, self.obs_thresh)
        f_mig = self._get_migration_force(my_x, my_y)

        # ----------------------------------------------------------------
        # Step 3: Adaptive Weight Scaling
        # ----------------------------------------------------------------
        eff_w_sep = self.w_sep
        eff_w_coh = self.w_coh
        eff_w_obs = self.w_obs

        if neighbours:
            # 1. Crowding Response (Bounded Separation Scaling)
            avg_neighbor_dist = sum(n[5] for n in neighbours) / len(neighbours)
            crowd_factor = max(0.0, 1.0 - (avg_neighbor_dist / self.ideal_sep))
            eff_w_sep = clamp(self.w_sep + self.k_sep * crowd_factor, self.min_sep_w, self.max_sep_w)

            # 2. Fragmentation Response (Multiplicative Cohesion Scaling)
            cx = sum(n[1] for n in neighbours) / len(neighbours)
            cy = sum(n[2] for n in neighbours) / len(neighbours)
            local_coh = math.hypot(my_x - cx, my_y - cy)
            spread_factor = max(0.0, local_coh - self.thresh_spread)
            eff_w_coh = clamp(self.w_coh * (1.0 + self.alpha_coh * spread_factor), self.min_coh_w, self.max_coh_w)

        # 3. Threat-Proximity Response (Safe Obstacle Scaling)
        if self.latest_scan and getattr(self.latest_scan, 'ranges', None):
            valid_ranges = [r for r in self.latest_scan.ranges if math.isfinite(r) and r > 0.0]
            if valid_ranges:
                min_laser_dist = min(valid_ranges)
                safe_dist = max(min_laser_dist, self.laser_eps)
                scale = min(self.max_obs_w / self.w_obs if self.w_obs > 0 else 1.0, self.obs_thresh / safe_dist)
                eff_w_obs = clamp(self.w_obs * scale, self.min_obs_w, self.max_obs_w)

        # ----------------------------------------------------------------
        # Step 4: Weighted sum
        # ----------------------------------------------------------------
        fx = (eff_w_sep * f_sep[0] +
              self.w_ali * f_ali[0] +
              eff_w_coh * f_coh[0] +
              eff_w_obs * f_obs[0] +
              self.w_mig * f_mig[0])

        fy = (eff_w_sep * f_sep[1] +
              self.w_ali * f_ali[1] +
              eff_w_coh * f_coh[1] +
              eff_w_obs * f_obs[1] +
              self.w_mig * f_mig[1])

        # Step 4: convert resultant force → (linear, angular) commands
        lin, ang = force_to_cmd_vel(fx, fy, my_theta, self.max_lin, self.max_ang)

        # Step 5: exponential low-pass filter to smooth jerky commands
        self._smooth_lin = (self.lpf_alpha * lin +
                    (1.0 - self.lpf_alpha) * self._smooth_lin)
        self._smooth_ang = (self.lpf_alpha * ang +
                    (1.0 - self.lpf_alpha) * self._smooth_ang)

        # Step 6: clamp and publish
        cmd = Twist()
        cmd.linear.x  = clamp(self._smooth_lin, -self.max_lin, self.max_lin)
        cmd.angular.z = clamp(self._smooth_ang, -self.max_ang, self.max_ang)
        self.cmd_pub.publish(cmd)

        # Step 7: advance waypoint when close enough
        self._advance_waypoint(my_x, my_y)

    # ====================================================================
    # Neighbour helper
    # ====================================================================

    def _get_valid_neighbours(
        self, my_x: float, my_y: float
    ) -> list:
        """
        Return a list of NeighbourEntry tuples (id, x, y, vx, vy, dist)
        for robots that are:
          • Within neighbour_radius
          • Not stale (data newer than STALE_TIMEOUT_S)
        """
        now = time.monotonic()
        result = []

        for rid, np in self.neighbour_poses.items():
            # Stale check
            if (now - np.timestamp) > STALE_TIMEOUT_S:
                continue

            dist = math.hypot(np.x - my_x, np.y - my_y)
            if dist > self.neighbour_r:
                continue

            nv = self.neighbour_vels.get(rid)
            if nv is not None and (now - nv.timestamp) <= STALE_TIMEOUT_S:
                vx, vy = nv.vx, nv.vy
            else:
                vx, vy = 0.0, 0.0

            result.append((rid, np.x, np.y, vx, vy, dist))

        return result

    # ====================================================================
    # Migration force
    # ====================================================================

    def _get_migration_force(
        self, my_x: float, my_y: float
    ) -> Tuple[float, float]:
        """Return normalised direction toward the current waypoint (or (0,0))."""
        if self.current_wp >= len(self.waypoints):
            # All waypoints reached — no migration force
            return (0.0, 0.0)
        gx, gy = self.waypoints[self.current_wp]
        return compute_migration(my_x, my_y, gx, gy)

    def _advance_waypoint(self, my_x: float, my_y: float) -> None:
        """Advance the waypoint pointer when the robot arrives close enough."""
        if self.current_wp >= len(self.waypoints):
            return
        gx, gy = self.waypoints[self.current_wp]
        if math.hypot(gx - my_x, gy - my_y) < self.wp_arrival_r:
            self.get_logger().info(
                f'robot_{self.robot_id} reached waypoint {self.current_wp} '
                f'({gx}, {gy}) → advancing')
            self.current_wp += 1


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
