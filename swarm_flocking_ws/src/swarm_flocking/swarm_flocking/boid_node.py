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

from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, PointStamped
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
        self._last_no_odom_warn: float = 0.0
        self._front_blocked_since: Optional[float] = None
        self._recover_until: float = 0.0
        self._recover_turn_dir: float = 1.0
        self._last_wp_dist: Optional[float] = None
        self._last_progress_time: float = time.monotonic()
        self._desync_until: float = 0.0
        self._start_time: float = time.monotonic()
        self._stall_wp: int = -1
        self._stall_count: int = 0
        self._last_desync_log_time: float = 0.0
        self._progress_wp_idx: int = -1
        self._monitor_waypoint: Optional[Tuple[float, float]] = None
        self._monitor_waypoint_ts: float = 0.0
        self.smooth_w_cohesion: float = self.w_coh
        self.smooth_w_separation: float = self.w_sep
        self._odom_mode_local_effective: Optional[bool] = None

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

        # Fallback odom topics used by some Gazebo Harmonic setups.
        self.create_subscription(
            Odometry, f'/model/robot_{self.robot_id}/odometry',
            self._odom_callback, SENSOR_QOS)
        self.create_subscription(
            Odometry, f'/model/robot_{self.robot_id}/odometry_with_covariance',
            self._odom_callback, SENSOR_QOS)

        self.create_subscription(
            LaserScan, f'{ns}/scan',
            self._scan_callback, SENSOR_QOS)

        self.create_subscription(
            PointStamped, '/flock/active_waypoint',
            self._monitor_waypoint_callback, STATE_QOS)

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
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('coordinated_mode_enable', True)
        self.declare_parameter('coordinated_desired_radius', 0.9)
        self.declare_parameter('coordinated_w_group_migration', 2.2)
        self.declare_parameter('coordinated_w_center', 3.0)
        self.declare_parameter('coordinated_w_alignment', 1.4)
        self.declare_parameter('coordinated_w_separation', 0.45)
        self.declare_parameter('coordinated_w_self_migration', 0.9)
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
        self.declare_parameter('regroup_spread_threshold', 1.4)
        self.declare_parameter('regroup_gain',       1.0)
        self.declare_parameter('max_regroup_w',      2.0)
        self.declare_parameter('front_slowdown_distance', 1.0)
        self.declare_parameter('min_front_speed_scale', 0.2)
        self.declare_parameter('front_fov_deg',      80.0)
        self.declare_parameter('front_stop_distance', 0.35)
        self.declare_parameter('escape_turn_rate',   2.0)
        self.declare_parameter('wall_follow_gain',   1.1)
        self.declare_parameter('wall_follow_max_w',  1.6)
        self.declare_parameter('stuck_front_time_s', 1.0)
        self.declare_parameter('stuck_speed_threshold', 0.03)
        self.declare_parameter('recover_reverse_speed', 0.06)
        self.declare_parameter('recover_turn_rate', 2.2)
        self.declare_parameter('recover_duration_s', 1.0)
        self.declare_parameter('goal_projection_min', 0.45)
        self.declare_parameter('goal_projection_gain', 1.2)
        self.declare_parameter('goal_projection_max_boost', 1.4)
        self.declare_parameter('goal_projection_obs_relax', 0.85)
        self.declare_parameter('waypoint_sync_fraction', 0.35)
        self.declare_parameter('waypoint_sync_radius_scale', 1.6)
        self.declare_parameter('waypoint_bottleneck_guard_enable', True)
        self.declare_parameter('waypoint_bottleneck_guard_margin', 0.25)
        self.declare_parameter('waypoint_guard_window_x', 1.2)
        self.declare_parameter('sync_enable', True)
        self.declare_parameter('sync_leader_id', 0)
        self.declare_parameter('sync_columns', 3)
        self.declare_parameter('sync_spacing_x', 0.8)
        self.declare_parameter('sync_spacing_y', 0.55)
        self.declare_parameter('sync_position_gain', 1.4)
        self.declare_parameter('sync_velocity_gain', 0.5)
        self.declare_parameter('sync_max_w', 2.2)
        self.declare_parameter('sync_obstacle_relax', 0.9)
        self.declare_parameter('sync_error_boost_gain', 0.6)
        self.declare_parameter('sync_error_boost_max', 1.8)
        self.declare_parameter('bottleneck_mode_enable', True)
        self.declare_parameter('bottleneck_center_x', 8.5)
        self.declare_parameter('bottleneck_center_y', 7.5)
        self.declare_parameter('bottleneck_zone_half_x', 3.2)
        self.declare_parameter('bottleneck_zone_half_y', 4.0)
        self.declare_parameter('bottleneck_sync_columns', 1)
        self.declare_parameter('bottleneck_sync_spacing_x', 0.65)
        self.declare_parameter('bottleneck_sync_spacing_y', 0.18)
        self.declare_parameter('bottleneck_sync_gain_mult', 1.8)
        self.declare_parameter('bottleneck_sep_scale', 0.75)
        self.declare_parameter('bottleneck_migration_boost', 1.15)
        self.declare_parameter('neighbour_front_angle_deg', 20.0)
        self.declare_parameter('neighbour_as_obstacle_distance', 1.3)
        self.declare_parameter('neighbour_body_clearance', 0.30)
        self.declare_parameter('neighbour_scan_match_tol', 0.22)
        self.declare_parameter('neighbour_obstacle_scale', 0.2)
        self.declare_parameter('startup_relax_s', 6.0)
        self.declare_parameter('progress_timeout_s', 4.0)
        self.declare_parameter('progress_min_delta', 0.12)
        self.declare_parameter('desync_duration_s', 2.0)
        self.declare_parameter('desync_sync_scale', 0.25)
        self.declare_parameter('desync_migration_boost', 1.4)
        self.declare_parameter('stall_escalation_gain', 0.25)
        self.declare_parameter('desync_log_cooldown_s', 12.0)
        self.declare_parameter('use_monitor_waypoint', False)
        self.declare_parameter('monitor_waypoint_timeout_s', 3.0)
        self.declare_parameter('context_bottleneck_min_scan', 0.8)
        self.declare_parameter('context_weight_lpf_alpha', 0.05)
        self.declare_parameter('force_floor_min_mag', 0.15)
        self.declare_parameter('force_floor_boost', 0.6)

        # Spawn position offset: Gazebo's odom starts at (0,0) per robot.
        # We add these offsets to convert odom-frame pose to world-frame pose.
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        # If true, /odom starts at (0,0) in robot-local frame and needs
        # (spawn_x, spawn_y) offset. If false, /odom is already world-frame.
        self.declare_parameter('odom_is_local', True)
        self.declare_parameter('auto_detect_odom_frame', True)
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
        self.enable_obstacle_avoidance = bool(self.get_parameter('enable_obstacle_avoidance').value)
        self.coordinated_mode_enable = bool(self.get_parameter('coordinated_mode_enable').value)
        self.coordinated_desired_radius = max(0.2, float(self.get_parameter('coordinated_desired_radius').value))
        self.coordinated_w_group_migration = max(0.0, float(self.get_parameter('coordinated_w_group_migration').value))
        self.coordinated_w_center = max(0.0, float(self.get_parameter('coordinated_w_center').value))
        self.coordinated_w_alignment = max(0.0, float(self.get_parameter('coordinated_w_alignment').value))
        self.coordinated_w_separation = max(0.0, float(self.get_parameter('coordinated_w_separation').value))
        self.coordinated_w_self_migration = max(0.0, float(self.get_parameter('coordinated_w_self_migration').value))
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
        self.auto_detect_odom_frame = bool(self.get_parameter('auto_detect_odom_frame').value)

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
        self.regroup_spread_thresh = float(self.get_parameter('regroup_spread_threshold').value)
        self.regroup_gain  = float(self.get_parameter('regroup_gain').value)
        self.max_regroup_w = float(self.get_parameter('max_regroup_w').value)
        self.front_slowdown_dist = float(self.get_parameter('front_slowdown_distance').value)
        self.min_front_speed_scale = float(self.get_parameter('min_front_speed_scale').value)
        self.front_fov_deg = float(self.get_parameter('front_fov_deg').value)
        self.front_stop_dist = float(self.get_parameter('front_stop_distance').value)
        self.escape_turn_rate = float(self.get_parameter('escape_turn_rate').value)
        self.wall_follow_gain = float(self.get_parameter('wall_follow_gain').value)
        self.wall_follow_max_w = float(self.get_parameter('wall_follow_max_w').value)
        self.stuck_front_time_s = float(self.get_parameter('stuck_front_time_s').value)
        self.stuck_speed_thresh = float(self.get_parameter('stuck_speed_threshold').value)
        self.recover_reverse_speed = float(self.get_parameter('recover_reverse_speed').value)
        self.recover_turn_rate = float(self.get_parameter('recover_turn_rate').value)
        self.recover_duration_s = float(self.get_parameter('recover_duration_s').value)
        self.goal_proj_min = float(self.get_parameter('goal_projection_min').value)
        self.goal_proj_gain = float(self.get_parameter('goal_projection_gain').value)
        self.goal_proj_max_boost = float(self.get_parameter('goal_projection_max_boost').value)
        self.goal_proj_obs_relax = float(self.get_parameter('goal_projection_obs_relax').value)
        self.wp_sync_fraction = float(self.get_parameter('waypoint_sync_fraction').value)
        self.wp_sync_radius_scale = float(self.get_parameter('waypoint_sync_radius_scale').value)
        self.wp_bneck_guard_enable = bool(self.get_parameter('waypoint_bottleneck_guard_enable').value)
        self.wp_bneck_guard_margin = float(self.get_parameter('waypoint_bottleneck_guard_margin').value)
        self.wp_guard_window_x = float(self.get_parameter('waypoint_guard_window_x').value)
        self.sync_enable = bool(self.get_parameter('sync_enable').value)
        self.sync_leader_id = int(self.get_parameter('sync_leader_id').value)
        self.sync_columns = max(1, int(self.get_parameter('sync_columns').value))
        self.sync_spacing_x = float(self.get_parameter('sync_spacing_x').value)
        self.sync_spacing_y = float(self.get_parameter('sync_spacing_y').value)
        self.sync_position_gain = float(self.get_parameter('sync_position_gain').value)
        self.sync_velocity_gain = float(self.get_parameter('sync_velocity_gain').value)
        self.sync_max_w = float(self.get_parameter('sync_max_w').value)
        self.sync_obstacle_relax = float(self.get_parameter('sync_obstacle_relax').value)
        self.sync_error_boost_gain = float(self.get_parameter('sync_error_boost_gain').value)
        self.sync_error_boost_max = float(self.get_parameter('sync_error_boost_max').value)
        self.bottleneck_mode_enable = bool(self.get_parameter('bottleneck_mode_enable').value)
        self.bneck_cx = float(self.get_parameter('bottleneck_center_x').value)
        self.bneck_cy = float(self.get_parameter('bottleneck_center_y').value)
        self.bneck_half_x = float(self.get_parameter('bottleneck_zone_half_x').value)
        self.bneck_half_y = float(self.get_parameter('bottleneck_zone_half_y').value)
        self.bneck_sync_columns = max(1, int(self.get_parameter('bottleneck_sync_columns').value))
        self.bneck_sync_spacing_x = float(self.get_parameter('bottleneck_sync_spacing_x').value)
        self.bneck_sync_spacing_y = float(self.get_parameter('bottleneck_sync_spacing_y').value)
        self.bneck_sync_gain_mult = float(self.get_parameter('bottleneck_sync_gain_mult').value)
        self.bneck_sep_scale = float(self.get_parameter('bottleneck_sep_scale').value)
        self.bneck_mig_boost = float(self.get_parameter('bottleneck_migration_boost').value)
        self.neighbour_front_angle_deg = float(self.get_parameter('neighbour_front_angle_deg').value)
        self.neighbour_as_obstacle_dist = float(self.get_parameter('neighbour_as_obstacle_distance').value)
        self.neighbour_body_clearance = float(self.get_parameter('neighbour_body_clearance').value)
        self.neighbour_scan_match_tol = float(self.get_parameter('neighbour_scan_match_tol').value)
        self.neighbour_obstacle_scale = float(self.get_parameter('neighbour_obstacle_scale').value)
        self.startup_relax_s = float(self.get_parameter('startup_relax_s').value)
        self.progress_timeout_s = float(self.get_parameter('progress_timeout_s').value)
        self.progress_min_delta = float(self.get_parameter('progress_min_delta').value)
        self.desync_duration_s = float(self.get_parameter('desync_duration_s').value)
        self.desync_sync_scale = float(self.get_parameter('desync_sync_scale').value)
        self.desync_migration_boost = float(self.get_parameter('desync_migration_boost').value)
        self.stall_escalation_gain = float(self.get_parameter('stall_escalation_gain').value)
        self.desync_log_cooldown_s = float(self.get_parameter('desync_log_cooldown_s').value)
        self.use_monitor_waypoint = bool(self.get_parameter('use_monitor_waypoint').value)
        self.monitor_waypoint_timeout_s = float(self.get_parameter('monitor_waypoint_timeout_s').value)
        self.context_bottleneck_min_scan = float(self.get_parameter('context_bottleneck_min_scan').value)
        self.context_weight_lpf_alpha = float(self.get_parameter('context_weight_lpf_alpha').value)
        self.force_floor_min_mag = float(self.get_parameter('force_floor_min_mag').value)
        self.force_floor_boost = float(self.get_parameter('force_floor_boost').value)

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
                elif name == 'enable_obstacle_avoidance':
                    self.enable_obstacle_avoidance = bool(value)
                elif name == 'coordinated_mode_enable':
                    self.coordinated_mode_enable = bool(value)
                elif name == 'coordinated_desired_radius':
                    self.coordinated_desired_radius = max(0.2, float(value))
                elif name == 'coordinated_w_group_migration':
                    self.coordinated_w_group_migration = max(0.0, float(value))
                elif name == 'coordinated_w_center':
                    self.coordinated_w_center = max(0.0, float(value))
                elif name == 'coordinated_w_alignment':
                    self.coordinated_w_alignment = max(0.0, float(value))
                elif name == 'coordinated_w_separation':
                    self.coordinated_w_separation = max(0.0, float(value))
                elif name == 'coordinated_w_self_migration':
                    self.coordinated_w_self_migration = max(0.0, float(value))
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
                elif name == 'regroup_spread_threshold':
                    self.regroup_spread_thresh = max(0.1, float(value))
                elif name == 'regroup_gain':
                    self.regroup_gain = max(0.0, float(value))
                elif name == 'max_regroup_w':
                    self.max_regroup_w = max(0.0, float(value))
                elif name == 'front_slowdown_distance':
                    self.front_slowdown_dist = max(0.05, float(value))
                elif name == 'min_front_speed_scale':
                    self.min_front_speed_scale = clamp(float(value), 0.0, 1.0)
                elif name == 'front_fov_deg':
                    self.front_fov_deg = clamp(float(value), 20.0, 180.0)
                elif name == 'front_stop_distance':
                    self.front_stop_dist = max(0.05, float(value))
                elif name == 'escape_turn_rate':
                    self.escape_turn_rate = max(0.1, float(value))
                elif name == 'wall_follow_gain':
                    self.wall_follow_gain = max(0.0, float(value))
                elif name == 'wall_follow_max_w':
                    self.wall_follow_max_w = max(0.0, float(value))
                elif name == 'stuck_front_time_s':
                    self.stuck_front_time_s = max(0.1, float(value))
                elif name == 'stuck_speed_threshold':
                    self.stuck_speed_thresh = max(0.0, float(value))
                elif name == 'recover_reverse_speed':
                    self.recover_reverse_speed = max(0.0, float(value))
                elif name == 'recover_turn_rate':
                    self.recover_turn_rate = max(0.1, float(value))
                elif name == 'recover_duration_s':
                    self.recover_duration_s = max(0.1, float(value))
                elif name == 'goal_projection_min':
                    self.goal_proj_min = max(0.0, float(value))
                elif name == 'goal_projection_gain':
                    self.goal_proj_gain = max(0.0, float(value))
                elif name == 'goal_projection_max_boost':
                    self.goal_proj_max_boost = max(0.0, float(value))
                elif name == 'goal_projection_obs_relax':
                    self.goal_proj_obs_relax = max(0.05, float(value))
                elif name == 'waypoint_sync_fraction':
                    self.wp_sync_fraction = clamp(float(value), 0.0, 1.0)
                elif name == 'waypoint_sync_radius_scale':
                    self.wp_sync_radius_scale = max(1.0, float(value))
                elif name == 'waypoint_bottleneck_guard_enable':
                    self.wp_bneck_guard_enable = bool(value)
                elif name == 'waypoint_bottleneck_guard_margin':
                    self.wp_bneck_guard_margin = clamp(float(value), 0.05, 1.5)
                elif name == 'waypoint_guard_window_x':
                    self.wp_guard_window_x = clamp(float(value), 0.2, 5.0)
                elif name == 'sync_enable':
                    self.sync_enable = bool(value)
                elif name == 'sync_leader_id':
                    self.sync_leader_id = int(value)
                elif name == 'sync_columns':
                    self.sync_columns = max(1, int(value))
                elif name == 'sync_spacing_x':
                    self.sync_spacing_x = max(0.05, float(value))
                elif name == 'sync_spacing_y':
                    self.sync_spacing_y = max(0.05, float(value))
                elif name == 'sync_position_gain':
                    self.sync_position_gain = max(0.0, float(value))
                elif name == 'sync_velocity_gain':
                    self.sync_velocity_gain = max(0.0, float(value))
                elif name == 'sync_max_w':
                    self.sync_max_w = max(0.0, float(value))
                elif name == 'sync_obstacle_relax':
                    self.sync_obstacle_relax = max(0.0, float(value))
                elif name == 'sync_error_boost_gain':
                    self.sync_error_boost_gain = max(0.0, float(value))
                elif name == 'sync_error_boost_max':
                    self.sync_error_boost_max = max(1.0, float(value))
                elif name == 'bottleneck_mode_enable':
                    self.bottleneck_mode_enable = bool(value)
                elif name == 'bottleneck_center_x':
                    self.bneck_cx = float(value)
                elif name == 'bottleneck_center_y':
                    self.bneck_cy = float(value)
                elif name == 'bottleneck_zone_half_x':
                    self.bneck_half_x = max(0.1, float(value))
                elif name == 'bottleneck_zone_half_y':
                    self.bneck_half_y = max(0.1, float(value))
                elif name == 'bottleneck_sync_columns':
                    self.bneck_sync_columns = max(1, int(value))
                elif name == 'bottleneck_sync_spacing_x':
                    self.bneck_sync_spacing_x = max(0.05, float(value))
                elif name == 'bottleneck_sync_spacing_y':
                    self.bneck_sync_spacing_y = max(0.0, float(value))
                elif name == 'bottleneck_sync_gain_mult':
                    self.bneck_sync_gain_mult = max(0.1, float(value))
                elif name == 'bottleneck_sep_scale':
                    self.bneck_sep_scale = clamp(float(value), 0.2, 1.5)
                elif name == 'bottleneck_migration_boost':
                    self.bneck_mig_boost = max(0.1, float(value))
                elif name == 'neighbour_front_angle_deg':
                    self.neighbour_front_angle_deg = clamp(float(value), 1.0, 90.0)
                elif name == 'neighbour_as_obstacle_distance':
                    self.neighbour_as_obstacle_dist = max(0.1, float(value))
                elif name == 'neighbour_body_clearance':
                    self.neighbour_body_clearance = max(0.0, float(value))
                elif name == 'neighbour_scan_match_tol':
                    self.neighbour_scan_match_tol = max(0.01, float(value))
                elif name == 'neighbour_obstacle_scale':
                    self.neighbour_obstacle_scale = clamp(float(value), 0.0, 1.0)
                elif name == 'startup_relax_s':
                    self.startup_relax_s = max(0.0, float(value))
                elif name == 'progress_timeout_s':
                    self.progress_timeout_s = max(0.5, float(value))
                elif name == 'progress_min_delta':
                    self.progress_min_delta = max(0.0, float(value))
                elif name == 'desync_duration_s':
                    self.desync_duration_s = max(0.1, float(value))
                elif name == 'desync_sync_scale':
                    self.desync_sync_scale = clamp(float(value), 0.0, 1.0)
                elif name == 'desync_migration_boost':
                    self.desync_migration_boost = max(1.0, float(value))
                elif name == 'stall_escalation_gain':
                    self.stall_escalation_gain = max(0.0, float(value))
                elif name == 'desync_log_cooldown_s':
                    self.desync_log_cooldown_s = max(0.0, float(value))
                elif name == 'use_monitor_waypoint':
                    self.use_monitor_waypoint = bool(value)
                elif name == 'monitor_waypoint_timeout_s':
                    self.monitor_waypoint_timeout_s = max(0.1, float(value))
                elif name == 'context_bottleneck_min_scan':
                    self.context_bottleneck_min_scan = clamp(float(value), 0.2, 2.5)
                elif name == 'context_weight_lpf_alpha':
                    self.context_weight_lpf_alpha = clamp(float(value), 0.01, 1.0)
                elif name == 'force_floor_min_mag':
                    self.force_floor_min_mag = clamp(float(value), 0.01, 2.0)
                elif name == 'force_floor_boost':
                    self.force_floor_boost = clamp(float(value), 0.0, 3.0)
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
                    self._odom_mode_local_effective = None
                elif name == 'auto_detect_odom_frame':
                    self.auto_detect_odom_frame = bool(value)
                    self._odom_mode_local_effective = None

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

        if self._odom_mode_local_effective is None:
            mode_local = self.odom_is_local
            if self.auto_detect_odom_frame:
                dist_to_spawn = math.hypot(pos.x - self.spawn_x, pos.y - self.spawn_y)
                dist_to_origin = math.hypot(pos.x, pos.y)
                spawn_norm = math.hypot(self.spawn_x, self.spawn_y)

                # If odom already matches world spawn coordinates, avoid double-offsetting.
                if dist_to_spawn <= 0.8 and spawn_norm > 1.0:
                    mode_local = False
                # If odom starts near origin while spawn is not near origin, offset is required.
                elif dist_to_origin <= 0.8 and spawn_norm > 1.0:
                    mode_local = True

            self._odom_mode_local_effective = mode_local
            mode_text = 'local+offset' if mode_local else 'world'
            self.get_logger().info(
                f'robot_{self.robot_id} odom frame resolved as {mode_text} '
                f'(configured odom_is_local={self.odom_is_local}, auto_detect={self.auto_detect_odom_frame})'
            )

        if self._odom_mode_local_effective:
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

    def _monitor_waypoint_callback(self, msg: PointStamped) -> None:
        """Cache monitor-published active waypoint for coordinated migration."""
        self._monitor_waypoint = (msg.point.x, msg.point.y)
        self._monitor_waypoint_ts = time.monotonic()

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
            # No odometry yet — hold still and emit periodic diagnostics.
            now = time.monotonic()
            if now - self._last_no_odom_warn > 5.0:
                self._last_no_odom_warn = now
                self.get_logger().warn(
                    f'No odom received yet on /robot_{self.robot_id}/odom; holding position.'
                )
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
        now = time.monotonic()
        obstacle_avoidance_enabled = self.enable_obstacle_avoidance

        # Dedicated open-field coordinated controller: keep flock compact while
        # moving group centroid toward the active waypoint.
        if (not obstacle_avoidance_enabled) and self.coordinated_mode_enable:
            self._run_coordinated_open_field(my_x, my_y, my_theta, my_vx, my_vy)
            return

        startup_phase = (now - self._start_time) < self.startup_relax_s
        in_bottleneck = self._in_bottleneck_zone(my_x, my_y)

        # Timed recovery mode to break local minima near bottlenecks/walls.
        if now < self._recover_until:
            cmd = Twist()
            cmd.linear.x = -min(self.recover_reverse_speed, self.max_lin)
            cmd.angular.z = clamp(self._recover_turn_dir * self.recover_turn_rate, -self.max_ang, self.max_ang)
            self._smooth_lin = 0.0
            self._smooth_ang = cmd.angular.z
            self.cmd_pub.publish(cmd)
            self._advance_waypoint(my_x, my_y)
            return

        # Step 1: collect valid, non-stale neighbours
        neighbours = self._get_valid_neighbours(my_x, my_y)
        scan_ranges = []
        if obstacle_avoidance_enabled and self.latest_scan and getattr(self.latest_scan, 'ranges', None):
            scan_ranges = list(self.latest_scan.ranges)
        context = self.compute_context(neighbours, scan_ranges)
        front_min = float('inf')
        nearest_front_nei = float('inf')
        front_is_neighbour = False
        if obstacle_avoidance_enabled:
            front_min = self._get_front_min_distance()
            nearest_front_nei = self._nearest_front_neighbour_distance(my_x, my_y, my_theta, neighbours)
            if (math.isfinite(front_min) and math.isfinite(nearest_front_nei) and
                    nearest_front_nei <= self.neighbour_as_obstacle_dist):
                expected_front = max(0.0, nearest_front_nei - self.neighbour_body_clearance)
                front_is_neighbour = abs(front_min - expected_front) <= self.neighbour_scan_match_tol
                if (not front_is_neighbour and startup_phase and
                        front_min <= max(self.front_stop_dist * 1.8, 0.45) and
                        nearest_front_nei <= self.neighbour_as_obstacle_dist):
                    front_is_neighbour = True

        # Step 2: compute each force component (all return unit vectors)
        f_sep = compute_separation(my_x, my_y, neighbours, self.sep_r)
        f_ali = compute_alignment(my_vx, my_vy, neighbours)
        f_coh = compute_cohesion(my_x, my_y, neighbours)
        f_obs = (0.0, 0.0)
        if obstacle_avoidance_enabled:
            f_obs = laser_to_repulsive_force(self.latest_scan, my_theta, self.obs_thresh)
        f_mig = self._get_migration_force(my_x, my_y)
        f_sep = (clamp(f_sep[0], -1.0, 1.0), clamp(f_sep[1], -1.0, 1.0))
        f_ali = (clamp(f_ali[0], -1.0, 1.0), clamp(f_ali[1], -1.0, 1.0))
        f_coh = (clamp(f_coh[0], -1.0, 1.0), clamp(f_coh[1], -1.0, 1.0))
        f_obs = (clamp(f_obs[0], -1.0, 1.0), clamp(f_obs[1], -1.0, 1.0))
        f_mig = (clamp(f_mig[0], -1.0, 1.0), clamp(f_mig[1], -1.0, 1.0))

        # ----------------------------------------------------------------
        # Step 3: Adaptive Weight Scaling
        # ----------------------------------------------------------------
        eff_w_sep = self.w_sep
        eff_w_ali = self.w_ali
        eff_w_coh = self.w_coh
        eff_w_obs = self.w_obs
        eff_w_mig = self.w_mig
        regroup_w = 0.0
        f_regroup = (0.0, 0.0)
        wall_follow_w = 0.0
        f_wall = (0.0, 0.0)
        sync_w = 0.0
        f_sync = (0.0, 0.0)
        wall_balance = 0.0
        obstacle_proximity = 0.0
        wp_dist = 0.0

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

            # 2b. Regroup mode when local spread gets too high.
            if local_coh > self.regroup_spread_thresh:
                spread_excess = local_coh - self.regroup_spread_thresh
                regroup_w = clamp(
                    self.regroup_gain * (spread_excess / max(self.regroup_spread_thresh, 0.1)),
                    0.0,
                    self.max_regroup_w,
                )
                f_regroup = compute_migration(my_x, my_y, cx, cy)
                # Soften separation while regrouping to reduce further breakup.
                sep_soften = clamp(1.0 - 0.35 * regroup_w / max(self.max_regroup_w, 1e-6), 0.6, 1.0)
                eff_w_sep *= sep_soften

        # 3. Threat-Proximity Response (Safe Obstacle Scaling)
        if obstacle_avoidance_enabled and self.latest_scan and getattr(self.latest_scan, 'ranges', None):
            valid_ranges = [r for r in self.latest_scan.ranges if math.isfinite(r) and r > 0.0]
            if valid_ranges:
                min_laser_dist = min(valid_ranges)
                safe_dist = max(min_laser_dist, self.laser_eps)
                proximity = clamp((self.obs_thresh - safe_dist) / max(self.obs_thresh, self.laser_eps), 0.0, 1.0)
                obstacle_proximity = proximity
                # Keep a migration floor near obstacles to avoid stall/spin lock.
                eff_w_mig = self.w_mig * (1.0 - 0.55 * proximity)
                eff_w_ali = self.w_ali * (1.0 + 0.35 * proximity)
                eff_w_coh = clamp(eff_w_coh * (1.0 + 0.45 * proximity), self.min_coh_w, self.max_coh_w)
                scale = min(self.max_obs_w / self.w_obs if self.w_obs > 0 else 1.0, self.obs_thresh / safe_dist)
                eff_w_obs = clamp(self.w_obs * scale, self.min_obs_w, self.max_obs_w)
                if front_is_neighbour:
                    eff_w_obs *= self.neighbour_obstacle_scale
                    obstacle_proximity *= self.neighbour_obstacle_scale
                if safe_dist <= self.front_stop_dist and not front_is_neighbour:
                    eff_w_obs = self.max_obs_w

                # Tangential wall-following: choose side that best aligns with goal direction.
                obs_mag = math.hypot(f_obs[0], f_obs[1])
                if obs_mag > 1e-6:
                    t1 = (-f_obs[1], f_obs[0])
                    t2 = (f_obs[1], -f_obs[0])
                    if (f_mig[0] != 0.0) or (f_mig[1] != 0.0):
                        dot1 = t1[0] * f_mig[0] + t1[1] * f_mig[1]
                        dot2 = t2[0] * f_mig[0] + t2[1] * f_mig[1]
                        tx, ty = t1 if dot1 >= dot2 else t2
                    else:
                        tx, ty = t1

                    tmag = math.hypot(tx, ty)
                    if tmag > 1e-6:
                        f_wall = (tx / tmag, ty / tmag)
                        wall_follow_w = clamp(self.wall_follow_gain * obstacle_proximity, 0.0, self.wall_follow_max_w)

        # When far from the active waypoint, increase migration pull to sustain progress.
        target_wp = self._get_active_waypoint_target()
        if target_wp is not None:
            gx, gy = target_wp
            wp_dist = math.hypot(gx - my_x, gy - my_y)
            eff_w_mig *= clamp(wp_dist / 6.0, 1.0, 1.8)

        if in_bottleneck:
            eff_w_sep *= self.bneck_sep_scale
            eff_w_mig *= self.bneck_mig_boost
            eff_w_coh = clamp(eff_w_coh * 1.15, self.min_coh_w, self.max_coh_w)

        # Context-specific adaptive multipliers (layered over existing base weights).
        sep_mult = 1.0
        coh_mult = 1.0
        mig_mult = 1.0
        obs_mult = 1.0
        if context == 'BOTTLENECK':
            coh_mult = 0.1
            sep_mult = 2.5
            mig_mult = 2.0
            obs_mult = 3.0
            wall_balance = self.compute_wall_balance_force(scan_ranges)
        elif context == 'FRAGMENTED':
            coh_mult = 3.0
            mig_mult = 0.5

        sep_mult = clamp(sep_mult, 0.0, 4.0)
        coh_mult = clamp(coh_mult, 0.0, 4.0)
        mig_mult = clamp(mig_mult, 0.0, 4.0)
        obs_mult = clamp(obs_mult, 0.0, 4.0)

        target_w_sep = clamp(eff_w_sep * sep_mult, 0.0, max(0.1, self.max_sep_w * 4.0))
        target_w_coh = clamp(eff_w_coh * coh_mult, 0.0, max(0.1, self.max_coh_w * 4.0))
        eff_w_mig = clamp(eff_w_mig * mig_mult, 0.0, max(0.1, self.w_mig * 6.0))
        eff_w_obs = clamp(eff_w_obs * obs_mult, 0.0, max(0.1, self.max_obs_w * 4.0))

        # Smooth weight transitions to avoid abrupt flock breakup at context boundaries.
        alpha = clamp(self.context_weight_lpf_alpha, 0.01, 1.0)
        self.smooth_w_separation += alpha * (target_w_sep - self.smooth_w_separation)
        self.smooth_w_cohesion += alpha * (target_w_coh - self.smooth_w_cohesion)
        eff_w_sep = clamp(self.smooth_w_separation, 0.0, max(0.1, self.max_sep_w * 4.0))
        eff_w_coh = clamp(self.smooth_w_cohesion, 0.0, max(0.1, self.max_coh_w * 4.0))

        # Detect local progress stalls and temporarily relax sync to unblock.
        speed_mag = math.hypot(my_vx, my_vy)
        if obstacle_avoidance_enabled:
            self._update_progress_state(wp_dist, now, speed_mag, front_min, front_is_neighbour)
        else:
            self._desync_until = 0.0
            self._stall_wp = -1
            self._stall_count = 0

        # Synchronize relative positions/velocity around leader unless obstacle pressure is high.
        f_sync, sync_w = self._compute_sync_force(
            my_x, my_y, my_theta, my_vx, my_vy, obstacle_proximity, in_bottleneck
        )
        f_sync = (clamp(f_sync[0], -1.0, 1.0), clamp(f_sync[1], -1.0, 1.0))
        if now < self._desync_until:
            stall_level = min(4, self._stall_count)
            sync_scale = self.desync_sync_scale / (1.0 + 0.2 * stall_level)
            mig_boost = self.desync_migration_boost * (1.0 + self.stall_escalation_gain * stall_level)
            sync_w *= clamp(sync_scale, 0.05, 1.0)
            eff_w_mig *= mig_boost

        # ----------------------------------------------------------------
        # Step 4: Weighted sum
        # ----------------------------------------------------------------
        fx = (eff_w_sep * f_sep[0] +
              eff_w_ali * f_ali[0] +
              eff_w_coh * f_coh[0] +
              eff_w_obs * f_obs[0] +
              eff_w_mig * f_mig[0] +
              regroup_w * f_regroup[0] +
              wall_follow_w * f_wall[0] +
              sync_w * f_sync[0])

        fy = (eff_w_sep * f_sep[1] +
              eff_w_ali * f_ali[1] +
              eff_w_coh * f_coh[1] +
              eff_w_obs * f_obs[1] +
              eff_w_mig * f_mig[1] +
              regroup_w * f_regroup[1] +
              wall_follow_w * f_wall[1] +
              sync_w * f_sync[1])

        if context == 'BOTTLENECK':
            # Project lateral correction to world frame (+left in robot frame).
            side_x = -math.sin(my_theta)
            side_y = math.cos(my_theta)
            fx += clamp(side_x * wall_balance, -1.0, 1.0)
            fy += clamp(side_y * wall_balance, -1.0, 1.0)

        # Cancellation guard: ensure a small migration-aligned component survives competing forces.
        net_mag = math.hypot(fx, fy)
        if target_wp is not None and (f_mig[0] != 0.0 or f_mig[1] != 0.0) and net_mag < self.force_floor_min_mag:
            fx += clamp(self.force_floor_boost * f_mig[0], -1.0, 1.0)
            fy += clamp(self.force_floor_boost * f_mig[1], -1.0, 1.0)

        fx = clamp(fx, -25.0, 25.0)
        fy = clamp(fy, -25.0, 25.0)

        # Enforce a minimum net component toward waypoint direction.
        if target_wp is not None and (f_mig[0] != 0.0 or f_mig[1] != 0.0):
            goal_dot = fx * f_mig[0] + fy * f_mig[1]
            dist_scale = clamp(wp_dist / 5.0, 0.9, 1.8)
            obs_ratio = obstacle_proximity / max(self.goal_proj_obs_relax, 0.05)
            obs_scale = clamp(1.0 - obs_ratio * obs_ratio, 0.0, 1.0)
            proj_floor = self.goal_proj_min * dist_scale * obs_scale
            if goal_dot < proj_floor:
                boost = clamp((proj_floor - goal_dot) * self.goal_proj_gain, 0.0, self.goal_proj_max_boost)
                fx += boost * f_mig[0]
                fy += boost * f_mig[1]

        # Step 4: convert resultant force → (linear, angular) commands
        lin, ang = force_to_cmd_vel(fx, fy, my_theta, self.max_lin, self.max_ang)

        # Step 5: exponential low-pass filter to smooth jerky commands
        self._smooth_lin = (self.lpf_alpha * lin +
                    (1.0 - self.lpf_alpha) * self._smooth_lin)
        self._smooth_ang = (self.lpf_alpha * ang +
                    (1.0 - self.lpf_alpha) * self._smooth_ang)

        # Step 6: clamp and publish
        lin_cmd = clamp(self._smooth_lin, -self.max_lin, self.max_lin)
        ang_cmd = clamp(self._smooth_ang, -self.max_ang, self.max_ang)

        if obstacle_avoidance_enabled:
            effective_front = front_min
            if front_is_neighbour:
                # Do not hard-stop on a teammate in front; keep gentle motion to avoid spawn deadlock.
                gap = max(0.0, nearest_front_nei - self.neighbour_body_clearance)
                desired_gap = max(0.18, 0.8 * self.front_stop_dist)
                if gap <= desired_gap:
                    effective_front = max(front_min, self.front_stop_dist + 0.05)
                else:
                    effective_front = float('inf')

            front_speed_scale = self._compute_front_speed_scale(effective_front)
            lin_cmd = clamp(self._smooth_lin * front_speed_scale, -self.max_lin, self.max_lin)

            # Hard safety near walls: stop forward motion and force turn toward freer side.
            if math.isfinite(front_min) and front_min <= self.front_stop_dist and not front_is_neighbour:
                lin_cmd = min(0.0, lin_cmd)
                turn_dir = self._compute_escape_turn_direction()
                desired_escape = turn_dir * self.escape_turn_rate
                if abs(ang_cmd) < abs(desired_escape):
                    ang_cmd = desired_escape

                if self._front_blocked_since is None:
                    self._front_blocked_since = now
                elif ((now - self._front_blocked_since) >= self.stuck_front_time_s and
                      speed_mag <= self.stuck_speed_thresh):
                    self._recover_turn_dir = turn_dir
                    self._recover_until = now + self.recover_duration_s
                    self._front_blocked_since = None
                    self.get_logger().info(
                        f'robot_{self.robot_id} recovery: blocked front={front_min:.2f}m, speed={speed_mag:.2f}m/s')
                    cmd = Twist()
                    cmd.linear.x = -min(self.recover_reverse_speed, self.max_lin)
                    cmd.angular.z = clamp(self._recover_turn_dir * self.recover_turn_rate, -self.max_ang, self.max_ang)
                    self._smooth_lin = 0.0
                    self._smooth_ang = cmd.angular.z
                    self.cmd_pub.publish(cmd)
                    self._advance_waypoint(my_x, my_y)
                    return
            else:
                self._front_blocked_since = None
        else:
            self._front_blocked_since = None

        cmd = Twist()
        cmd.linear.x  = lin_cmd
        cmd.angular.z = clamp(ang_cmd, -self.max_ang, self.max_ang)
        self.cmd_pub.publish(cmd)

        # Step 7: advance waypoint when close enough
        self._advance_waypoint(my_x, my_y)

    def _run_coordinated_open_field(
        self,
        my_x: float,
        my_y: float,
        my_theta: float,
        my_vx: float,
        my_vy: float,
    ) -> None:
        """Compact-group controller used for no-obstacle coordinated runs."""
        neighbours = self._get_active_neighbours_global(my_x, my_y)
        target_wp = self._get_active_waypoint_target()
        if target_wp is None:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        gx, gy = target_wp

        points = [(my_x, my_y)] + [(n[1], n[2]) for n in neighbours]
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        spread = sum(math.hypot(px - cx, py - cy) for px, py in points) / len(points)

        f_group = compute_migration(cx, cy, gx, gy)
        f_self = compute_migration(my_x, my_y, gx, gy)
        f_center = self._normalize_vec(cx - my_x, cy - my_y)
        f_sep = compute_separation(my_x, my_y, neighbours, self.sep_r)
        f_ali = compute_alignment(my_vx, my_vy, neighbours)

        # If flock spread grows, prioritize regrouping and soften migration.
        spread_ratio = max(0.0, (spread - self.coordinated_desired_radius) / max(0.2, self.coordinated_desired_radius))
        regroup_boost = 1.0 + min(2.0, 1.8 * spread_ratio)
        mig_scale = clamp(1.0 - 0.45 * min(1.0, spread_ratio), 0.45, 1.0)

        fx = (
            self.coordinated_w_group_migration * mig_scale * f_group[0] +
            self.coordinated_w_self_migration * f_self[0] +
            self.coordinated_w_center * regroup_boost * f_center[0] +
            self.coordinated_w_alignment * f_ali[0] +
            self.coordinated_w_separation * f_sep[0]
        )
        fy = (
            self.coordinated_w_group_migration * mig_scale * f_group[1] +
            self.coordinated_w_self_migration * f_self[1] +
            self.coordinated_w_center * regroup_boost * f_center[1] +
            self.coordinated_w_alignment * f_ali[1] +
            self.coordinated_w_separation * f_sep[1]
        )

        lin, ang = force_to_cmd_vel(fx, fy, my_theta, self.max_lin, self.max_ang)
        lin = max(0.0, lin)
        # Slow translation when spread is high so regroup can happen before further drift.
        speed_scale = clamp(1.0 - 0.55 * min(1.0, spread_ratio), 0.35, 1.0)
        lin *= speed_scale
        self._smooth_lin = (self.lpf_alpha * lin + (1.0 - self.lpf_alpha) * self._smooth_lin)
        self._smooth_ang = (self.lpf_alpha * ang + (1.0 - self.lpf_alpha) * self._smooth_ang)

        cmd = Twist()
        cmd.linear.x = clamp(self._smooth_lin, 0.0, self.max_lin)
        cmd.angular.z = clamp(self._smooth_ang, -0.75 * self.max_ang, 0.75 * self.max_ang)
        self.cmd_pub.publish(cmd)

        self._desync_until = 0.0
        self._stall_wp = -1
        self._stall_count = 0
        self._advance_waypoint(my_x, my_y)

    def _get_active_neighbours_global(self, my_x: float, my_y: float) -> list:
        """Return all fresh neighbours regardless of distance (for global cohesion mode)."""
        now = time.monotonic()
        result = []

        for rid, np in self.neighbour_poses.items():
            if (now - np.timestamp) > STALE_TIMEOUT_S:
                continue

            dist = math.hypot(np.x - my_x, np.y - my_y)
            nv = self.neighbour_vels.get(rid)
            if nv is not None and (now - nv.timestamp) <= STALE_TIMEOUT_S:
                vx, vy = nv.vx, nv.vy
            else:
                vx, vy = 0.0, 0.0

            result.append((rid, np.x, np.y, vx, vy, dist))

        return result

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
        target_wp = self._get_active_waypoint_target()
        if target_wp is None:
            # All waypoints reached — no migration force
            return (0.0, 0.0)
        gx, gy = target_wp
        return compute_migration(my_x, my_y, gx, gy)

    def _get_active_waypoint_target(self) -> Optional[Tuple[float, float]]:
        """Choose monitor-published waypoint when fresh, else fallback to local sequence."""
        if self.use_monitor_waypoint and self._monitor_waypoint is not None:
            age = time.monotonic() - self._monitor_waypoint_ts
            if age <= self.monitor_waypoint_timeout_s:
                return self._monitor_waypoint

        if self.current_wp >= len(self.waypoints):
            return None
        return self.waypoints[self.current_wp]

    def _advance_waypoint(self, my_x: float, my_y: float) -> None:
        """Advance the waypoint pointer when the robot arrives close enough."""
        if self.use_monitor_waypoint and self._monitor_waypoint is not None:
            age = time.monotonic() - self._monitor_waypoint_ts
            if age <= self.monitor_waypoint_timeout_s:
                return

        if self.current_wp >= len(self.waypoints):
            return
        gx, gy = self.waypoints[self.current_wp]
        if math.hypot(gx - my_x, gy - my_y) < self.wp_arrival_r:
            if not self._passes_waypoint_bottleneck_guard(my_x, gx):
                return
            active_neigh = self._count_active_neighbours()
            if active_neigh > 0:
                required = max(1, int(math.ceil(active_neigh * self.wp_sync_fraction)))
                sync_radius = self.wp_arrival_r * self.wp_sync_radius_scale
                near_count = self._count_neighbours_near_waypoint(gx, gy, sync_radius)
                if near_count < required:
                    return
            self.get_logger().info(
                f'robot_{self.robot_id} reached waypoint {self.current_wp} '
                f'({gx}, {gy}) → advancing')
            self.current_wp += 1
            # Reset progress baseline for the new waypoint to avoid false stall escalation.
            self._progress_wp_idx = -1
            self._last_wp_dist = None
            self._last_progress_time = time.monotonic()
            self._desync_until = 0.0
            self._stall_wp = -1
            self._stall_count = 0

    def _count_active_neighbours(self) -> int:
        """Count fresh neighbour poses irrespective of distance."""
        now = time.monotonic()
        return sum(1 for np in self.neighbour_poses.values() if (now - np.timestamp) <= STALE_TIMEOUT_S)

    def _count_neighbours_near_waypoint(self, gx: float, gy: float, radius: float) -> int:
        """Count fresh neighbours currently near the active waypoint."""
        now = time.monotonic()
        count = 0
        for np in self.neighbour_poses.values():
            if (now - np.timestamp) > STALE_TIMEOUT_S:
                continue
            if not self._passes_waypoint_bottleneck_guard(np.x, gx):
                continue
            if math.hypot(np.x - gx, np.y - gy) <= radius:
                count += 1
        return count

    def _passes_waypoint_bottleneck_guard(self, robot_x: float, waypoint_x: float) -> bool:
        """Reject waypoint completion when robot is on wrong side of bottleneck wall."""
        if not self.wp_bneck_guard_enable or not self.bottleneck_mode_enable:
            return True

        if abs(waypoint_x - self.bneck_cx) > self.wp_guard_window_x:
            return True

        margin = self.wp_bneck_guard_margin
        if waypoint_x >= (self.bneck_cx + margin):
            return robot_x >= (self.bneck_cx + margin)
        if waypoint_x <= (self.bneck_cx - margin):
            return robot_x <= (self.bneck_cx - margin)
        return True

    def _normalize_vec(self, x: float, y: float) -> Tuple[float, float]:
        """Safely normalize a 2-D vector."""
        mag = math.hypot(x, y)
        if mag < 1e-6:
            return (0.0, 0.0)
        return (x / mag, y / mag)

    def _formation_slot_offset(
        self,
        robot_id: int,
        columns: Optional[int] = None,
        spacing_x: Optional[float] = None,
        spacing_y: Optional[float] = None,
    ) -> Tuple[float, float]:
        """Desired local-frame slot offset (x forward, y left) behind the leader."""
        if robot_id == self.sync_leader_id:
            return (0.0, 0.0)
        cols = max(1, columns if columns is not None else self.sync_columns)
        sx = spacing_x if spacing_x is not None else self.sync_spacing_x
        sy = spacing_y if spacing_y is not None else self.sync_spacing_y
        seq = robot_id if robot_id < self.sync_leader_id else (robot_id - 1)
        row = (seq // cols) + 1
        col = seq % cols
        y_center = 0.5 * (cols - 1)
        local_x = -row * sx
        local_y = (col - y_center) * sy
        return (local_x, local_y)

    def _in_bottleneck_zone(self, x: float, y: float) -> bool:
        """Detect when robot is near the narrow bottleneck corridor."""
        if not self.bottleneck_mode_enable:
            return False
        return (abs(x - self.bneck_cx) <= self.bneck_half_x and
                abs(y - self.bneck_cy) <= self.bneck_half_y)

    def _compute_sync_force(
        self,
        my_x: float,
        my_y: float,
        my_theta: float,
        my_vx: float,
        my_vy: float,
        obstacle_proximity: float,
        in_bottleneck: bool,
    ) -> Tuple[Tuple[float, float], float]:
        """Compute leader-referenced sync force and effective weight."""
        if not self.sync_enable or self.robot_id == self.sync_leader_id:
            return ((0.0, 0.0), 0.0)

        now = time.monotonic()
        leader_pose = self.neighbour_poses.get(self.sync_leader_id)
        if leader_pose is None or (now - leader_pose.timestamp) > STALE_TIMEOUT_S:
            return ((0.0, 0.0), 0.0)

        leader_vel = self.neighbour_vels.get(self.sync_leader_id)
        if leader_vel is not None and (now - leader_vel.timestamp) <= STALE_TIMEOUT_S:
            leader_vx, leader_vy = leader_vel.vx, leader_vel.vy
        else:
            leader_vx, leader_vy = 0.0, 0.0

        sync_cols = self.sync_columns
        sync_sx = self.sync_spacing_x
        sync_sy = self.sync_spacing_y
        sync_gain = self.sync_position_gain

        if in_bottleneck or self._in_bottleneck_zone(leader_pose.x, leader_pose.y):
            sync_cols = self.bneck_sync_columns
            sync_sx = self.bneck_sync_spacing_x
            sync_sy = self.bneck_sync_spacing_y
            sync_gain *= self.bneck_sync_gain_mult

        local_x, local_y = self._formation_slot_offset(self.robot_id, sync_cols, sync_sx, sync_sy)
        c = math.cos(leader_pose.theta)
        s = math.sin(leader_pose.theta)
        target_x = leader_pose.x + (c * local_x - s * local_y)
        target_y = leader_pose.y + (s * local_x + c * local_y)
        pos_err = math.hypot(target_x - my_x, target_y - my_y)

        pos_fx, pos_fy = self._normalize_vec(target_x - my_x, target_y - my_y)
        vel_fx, vel_fy = self._normalize_vec(leader_vx - my_vx, leader_vy - my_vy)
        sync_fx, sync_fy = self._normalize_vec(
            pos_fx + self.sync_velocity_gain * vel_fx,
            pos_fy + self.sync_velocity_gain * vel_fy,
        )
        if sync_fx == 0.0 and sync_fy == 0.0:
            return ((0.0, 0.0), 0.0)

        obs_scale = clamp(1.0 - self.sync_obstacle_relax * obstacle_proximity, 0.25, 1.0)
        err_boost = clamp(1.0 + self.sync_error_boost_gain * pos_err, 1.0, self.sync_error_boost_max)
        sync_w = clamp(sync_gain * obs_scale * err_boost, 0.0, self.sync_max_w)
        return ((sync_fx, sync_fy), sync_w)

    def _nearest_front_neighbour_distance(
        self,
        my_x: float,
        my_y: float,
        my_theta: float,
        neighbours: list,
    ) -> float:
        """Nearest neighbour distance in a narrow front cone."""
        half_angle = math.radians(self.neighbour_front_angle_deg)
        nearest = float('inf')
        for (_rid, nx, ny, _vx, _vy, dist) in neighbours:
            bearing = math.atan2(ny - my_y, nx - my_x) - my_theta
            # Wrap to [-pi, pi]
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if abs(bearing) <= half_angle:
                nearest = min(nearest, dist)
        return nearest

    def _update_progress_state(
        self,
        wp_dist: float,
        now: float,
        speed_mag: float,
        front_min: float,
        front_is_neighbour: bool,
    ) -> None:
        """Track progress to waypoint and trigger temporary desync when stalled."""
        if self.current_wp >= len(self.waypoints):
            self._last_wp_dist = None
            self._desync_until = 0.0
            self._stall_wp = -1
            self._stall_count = 0
            self._progress_wp_idx = -1
            return

        # Reset progress reference whenever we start tracking a different waypoint.
        if self._progress_wp_idx != self.current_wp or self._last_wp_dist is None:
            self._progress_wp_idx = self.current_wp
            self._last_wp_dist = wp_dist
            self._last_progress_time = now
            return

        improvement = self._last_wp_dist - wp_dist
        relaxed_progress = max(0.02, 0.35 * self.progress_min_delta)

        if improvement >= self.progress_min_delta:
            self._last_progress_time = now
            self._stall_count = max(0, self._stall_count - 1)
            self._last_wp_dist = wp_dist
        elif improvement >= relaxed_progress:
            # Slow but monotonic progress should postpone desync assist.
            self._last_progress_time = now
            self._last_wp_dist = wp_dist

        if wp_dist <= (1.5 * self.wp_arrival_r):
            self._last_progress_time = now
            self._stall_count = 0
            return

        blocked_front = (
            math.isfinite(front_min) and
            front_min <= self.front_stop_dist and
            not front_is_neighbour
        )
        moving_enough = speed_mag >= max(0.015, 0.6 * self.stuck_speed_thresh)
        tiny_progress = max(0.005, 0.12 * relaxed_progress)
        moving_toward_goal = improvement >= tiny_progress

        if (now - self._last_progress_time) >= self.progress_timeout_s and now >= self._desync_until:
            if (not blocked_front) and moving_enough and moving_toward_goal:
                # Robot is still reducing waypoint distance; avoid false positive stall escalation.
                self._last_progress_time = now
                return

            if self.current_wp == self._stall_wp:
                self._stall_count += 1
            else:
                self._stall_wp = self.current_wp
                self._stall_count = 1

            duration = self.desync_duration_s * (1.0 + 0.3 * min(self._stall_count, 4))
            self._desync_until = now + duration
            self._last_progress_time = now
            if (now - self._last_desync_log_time) >= self.desync_log_cooldown_s:
                self._last_desync_log_time = now
                self.get_logger().info(
                    f'robot_{self.robot_id} desync assist: stalled at wp={self.current_wp}, '
                    f'dist={wp_dist:.2f}m, level={self._stall_count}, '
                    f'blocked={int(blocked_front)}, speed={speed_mag:.2f}')

    def _get_front_min_distance(self) -> float:
        """Return minimum finite front-beam distance within configured FOV."""
        scan = self.latest_scan
        if scan is None or not getattr(scan, 'ranges', None):
            return float('inf')

        half_fov = math.radians(max(1.0, self.front_fov_deg) * 0.5)
        min_front = float('inf')

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) <= half_fov:
                min_front = min(min_front, r)

        return min_front

    def _compute_escape_turn_direction(self) -> float:
        """Pick a turn direction toward the side with larger nearby clearance."""
        scan = self.latest_scan
        if scan is None or not getattr(scan, 'ranges', None):
            return 1.0

        left_min = float('inf')
        right_min = float('inf')

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if 0.3 <= angle <= (math.pi / 2.0):
                left_min = min(left_min, r)
            elif (-math.pi / 2.0) <= angle <= -0.3:
                right_min = min(right_min, r)

        if not math.isfinite(left_min) and not math.isfinite(right_min):
            return 1.0
        if not math.isfinite(left_min):
            return -1.0
        if not math.isfinite(right_min):
            return 1.0
        return 1.0 if left_min >= right_min else -1.0

    def _compute_front_speed_scale(self, front_min: Optional[float] = None) -> float:
        """Return [min_front_speed_scale, 1.0] based on nearest obstacle ahead."""
        if front_min is None:
            front_min = self._get_front_min_distance()

        if not math.isfinite(front_min):
            return 1.0

        if front_min <= self.front_stop_dist:
            return 0.0

        proximity = clamp(
            (self.front_slowdown_dist - front_min) / max(self.front_slowdown_dist, self.laser_eps),
            0.0,
            1.0,
        )
        return clamp(1.0 - proximity * (1.0 - self.min_front_speed_scale), self.min_front_speed_scale, 1.0)

    def compute_context(self, neighbors, scan_ranges) -> str:
        """Classify local control context: BOTTLENECK, FRAGMENTED, or OPEN_FIELD."""
        if not scan_ranges:
            return 'OPEN_FIELD'

        valid_ranges = [r for r in scan_ranges if math.isfinite(r) and r > 0.0]
        if not valid_ranges:
            return 'OPEN_FIELD'

        if min(valid_ranges) < self.context_bottleneck_min_scan:
            return 'BOTTLENECK'
        if len(neighbors) < 2:
            return 'FRAGMENTED'
        return 'OPEN_FIELD'

    def get_sector_min(self, ranges, angle_start_deg, angle_end_deg) -> float:
        """Return minimum valid range within a degree sector for any scan resolution."""
        n = len(ranges)
        if n == 0:
            return float('inf')

        a0 = float(angle_start_deg) % 360.0
        a1 = float(angle_end_deg) % 360.0
        i_start = int((a0 / 360.0) * n)
        i_end = int((a1 / 360.0) * n)

        if i_start == i_end:
            i_end = min(n, i_start + 1)

        if i_start < i_end:
            sector = ranges[i_start:i_end]
        else:
            sector = ranges[i_start:] + ranges[:i_end]

        valid = [r for r in sector if math.isfinite(r) and r > 0.05]
        return min(valid) if valid else float('inf')

    def compute_wall_balance_force(self, scan_ranges) -> float:
        """Compute bounded lateral offset from left/right wall distance mismatch."""
        if not scan_ranges:
            return 0.0

        left_dist = self.get_sector_min(scan_ranges, 60.0, 120.0)
        right_dist = self.get_sector_min(scan_ranges, 240.0, 300.0)
        if not math.isfinite(left_dist) or not math.isfinite(right_dist):
            return 0.0

        lateral = (left_dist - right_dist) * 0.5
        return clamp(lateral, -1.0, 1.0)


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
