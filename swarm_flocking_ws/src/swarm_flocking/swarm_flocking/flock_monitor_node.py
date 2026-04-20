#!/usr/bin/env python3
# FILE: swarm_flocking/flock_monitor_node.py
"""
FlockMonitorNode — singleton node that monitors the entire swarm.

Responsibilities:
  • Subscribe to all /robot_i/pose_share and /robot_i/velocity_share topics.
  • At 2 Hz:
      - Compute centroid and cohesion radius.
      - Compute average speed.
      - Graph-based split detection (BFS connected components).
      - Convex hull of all robot positions (Graham scan).
      - Detect near-collision pairs (for collision_count).
  • Publish:
      - /flock/centroid        (geometry_msgs/PointStamped)
      - /flock/convex_hull     (visualization_msgs/Marker  — LINE_STRIP)
      - /flock/neighbor_links  (visualization_msgs/MarkerArray — thin lines)
      - /flock/state           (swarm_interfaces/FlockState)

All geometry uses frame_id='map'.
"""

import math
import time
import csv
import os
import re
import subprocess
from collections import deque
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist

from swarm_interfaces.msg import FlockState
from swarm_flocking.utils.reynolds import yaw_from_quaternion

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

STALE_TIMEOUT_S = 2.0

# Near-collision threshold: if two robots are closer than this we count it
COLLISION_RADIUS = 0.25   # meters (slightly > TurtleBot3 burger body radius)

STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# Placeholder path sequence for bottleneck traversal.
# NOTE: These coordinates must be adjusted to match the actual world layout.
WAYPOINTS = [
    (0.0, 0.0),
    (-2.0, 0.0),
    (-4.5, 0.0),
    (-6.0, 0.0),
    (-8.5, 2.5),
]


# ---------------------------------------------------------------------------
# Simple data holders
# ---------------------------------------------------------------------------

class RobotState:
    __slots__ = ('x', 'y', 'theta', 'vx', 'vy', 'pose_ts', 'vel_ts')

    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.vx = self.vy = 0.0
        self.pose_ts: float = 0.0
        self.vel_ts: float = 0.0


# ---------------------------------------------------------------------------
# FlockMonitorNode
# ---------------------------------------------------------------------------

class FlockMonitorNode(Node):

    def __init__(self):
        super().__init__('flock_monitor')

        # ------- Parameters -------
        self.declare_parameter('num_robots', 6)
        self.declare_parameter('neighbor_radius', 3.0)
        self.declare_parameter('separation_radius', 0.8)
        default_waypoints = [coord for wp in WAYPOINTS for coord in wp]
        self.declare_parameter('waypoints', default_waypoints)
        self.declare_parameter('monitor_rate_hz', 2.0)
        self.declare_parameter('rolling_window_s', 10.0)
        self.declare_parameter('goal_tolerance', 1.2)
        self.declare_parameter('waypoint_reach_radius', 1.5)
        self.declare_parameter('waypoint_reach_fraction', 0.6)
        self.declare_parameter('split_recovery_trigger_s', 3.0)
        self.declare_parameter('split_recovery_duration_s', 5.0)
        self.declare_parameter('emergency_w_cohesion', 4.0)
        self.declare_parameter('emergency_w_migration', 0.2)
        self.declare_parameter('default_w_cohesion_restore', 1.0)
        self.declare_parameter('default_w_migration_restore', 0.3)
        self.declare_parameter('success_timeout_s', 300.0)
        self.declare_parameter('success_max_collision_rate', 0.20)
        self.declare_parameter('success_max_mean_cohesion', 3.5)
        self.declare_parameter('auto_shutdown_on_completion', True)
        self.declare_parameter('csv_filename', 'experiment_results.csv')
        self.declare_parameter('log_interval_s', 5.0)

        self.num_robots    = int(self.get_parameter('num_robots').value)
        self.neighbour_r   = float(self.get_parameter('neighbor_radius').value)
        self.monitor_rate_hz = max(0.1, float(self.get_parameter('monitor_rate_hz').value))
        self.rolling_window_s = max(1.0, float(self.get_parameter('rolling_window_s').value))
        self.goal_tolerance = max(0.1, float(self.get_parameter('goal_tolerance').value))
        self.waypoint_reach_radius = max(0.1, float(self.get_parameter('waypoint_reach_radius').value))
        self.waypoint_reach_fraction = min(1.0, max(0.1, float(self.get_parameter('waypoint_reach_fraction').value)))
        self.split_recovery_trigger_s = max(0.5, float(self.get_parameter('split_recovery_trigger_s').value))
        self.split_recovery_duration_s = max(0.5, float(self.get_parameter('split_recovery_duration_s').value))
        self.emergency_w_cohesion = max(0.0, float(self.get_parameter('emergency_w_cohesion').value))
        self.emergency_w_migration = max(0.0, float(self.get_parameter('emergency_w_migration').value))
        self.default_w_cohesion_restore = max(0.0, float(self.get_parameter('default_w_cohesion_restore').value))
        self.default_w_migration_restore = max(0.0, float(self.get_parameter('default_w_migration_restore').value))
        self.success_timeout_s = max(1.0, float(self.get_parameter('success_timeout_s').value))
        self.success_max_collision_rate = max(0.0, float(self.get_parameter('success_max_collision_rate').value))
        self.success_max_mean_cohesion = max(0.0, float(self.get_parameter('success_max_mean_cohesion').value))
        self.auto_shutdown_on_completion = bool(self.get_parameter('auto_shutdown_on_completion').value)
        self.csv_filename = str(self.get_parameter('csv_filename').value)
        self.log_interval_s = max(1.0, float(self.get_parameter('log_interval_s').value))

        flat = list(self.get_parameter('waypoints').value)
        if len(flat) % 2 != 0:
            flat = flat[:-1]
        self.waypoints = [(flat[k], flat[k+1]) for k in range(0, len(flat), 2)]
        self.final_waypoint = self.waypoints[-1] if self.waypoints else None
        self.current_waypoint_index = 0
        self.waypoints_completed = 0

        # ------- Goal/Metrics tracking state -------
        self.start_time = None
        self.total_steps = 0
        self.sum_cohesion = 0.0
        self.experiment_completed = False
        self.goal_reached = False
        self.experiment_success = False
        self.completion_reason = 'running'
        self._shutdown_timer = None
        self._last_log_time = 0.0
        self.split_timer = 0.0
        self.split_events_count = 0
        self._split_started_at = None
        self._emergency_active = False
        self._emergency_restore_at = 0.0
        self._emergency_original_weights: Dict[int, Tuple[float, float]] = {}
        self._boid_node_name_cache: Dict[int, str] = {}

        # Rolling metric windows
        self._collision_events_window: deque = deque()
        self._cohesion_window: deque = deque()

        # ------- Per-robot state -------
        self.robot_states: Dict[int, RobotState] = {
            i: RobotState() for i in range(self.num_robots)
        }

        # Cumulative collision count (persistent across cycles)
        self._cumulative_collisions: int = 0
        self._prev_collision_pairs: set = set()

        # ------- Subscriptions -------
        for i in range(self.num_robots):
            self.create_subscription(
                PoseStamped,
                f'/robot_{i}/pose_share',
                lambda msg, rid=i: self._pose_cb(rid, msg),
                STATE_QOS,
            )
            self.create_subscription(
                TwistStamped,
                f'/robot_{i}/velocity_share',
                lambda msg, rid=i: self._vel_cb(rid, msg),
                STATE_QOS,
            )

        # ------- Publishers -------
        self.centroid_pub = self.create_publisher(
            PointStamped, '/flock/centroid', 10)
        self.hull_pub = self.create_publisher(
            Marker, '/flock/convex_hull', 10)
        self.links_pub = self.create_publisher(
            MarkerArray, '/flock/neighbor_links', 10)
        self.state_pub = self.create_publisher(
            FlockState, '/flock/state', 10)
        self.active_waypoint_pub = self.create_publisher(
            PointStamped, '/flock/active_waypoint', 10)

        # Command publishers for safe termination
        self.cmd_pubs = {}
        for i in range(self.num_robots):
            self.cmd_pubs[i] = self.create_publisher(Twist, f'/robot_{i}/cmd_vel', STATE_QOS)

        # ------- Timer -------
        self.timer = self.create_timer(1.0 / self.monitor_rate_hz, self._compute_and_publish)

        self.get_logger().info(
            f'FlockMonitorNode started — monitoring {self.num_robots} robots at {self.monitor_rate_hz:.2f} Hz')

    def _safe_shutdown_context(self) -> None:
        """Shutdown default context if still valid; ignore teardown races."""
        try:
            if rclpy.ok(context=self.context):
                rclpy.shutdown(context=self.context)
        except Exception:
            pass

    # ====================================================================
    # Callbacks
    # ====================================================================

    def _pose_cb(self, robot_id: int, msg: PoseStamped) -> None:
        s = self.robot_states[robot_id]
        s.x = msg.pose.position.x
        s.y = msg.pose.position.y
        s.theta = yaw_from_quaternion(msg.pose.orientation)
        s.pose_ts = time.monotonic()

    def _vel_cb(self, robot_id: int, msg: TwistStamped) -> None:
        s = self.robot_states[robot_id]
        s.vx = msg.twist.linear.x
        s.vy = msg.twist.linear.y
        s.vel_ts = time.monotonic()

    # ====================================================================
    # Main compute cycle
    # ====================================================================

    def _compute_and_publish(self) -> None:
        if self.experiment_completed:
            return

        now = time.monotonic()
        if self.start_time is None:
            self.start_time = now

        stamp = self.get_clock().now().to_msg()

        # Filter to robots with fresh pose data
        active_ids = [
            rid for rid, s in self.robot_states.items()
            if s.pose_ts > 0.0 and (now - s.pose_ts) <= STALE_TIMEOUT_S
        ]

        num_active = len(active_ids)
        if num_active == 0:
            return

        # ------------------------------------------------------------------
        # 1. Centroid
        # ------------------------------------------------------------------
        cx = sum(self.robot_states[rid].x for rid in active_ids) / num_active
        cy = sum(self.robot_states[rid].y for rid in active_ids) / num_active

        # ------------------------------------------------------------------
        # 2. Cohesion radius (mean distance from centroid)
        # ------------------------------------------------------------------
        cohesion_radius = sum(
            math.hypot(self.robot_states[rid].x - cx,
                       self.robot_states[rid].y - cy)
            for rid in active_ids
        ) / num_active

        self.sum_cohesion += cohesion_radius
        self.total_steps += 1

        # ------------------------------------------------------------------
        # 3. Average speed
        # ------------------------------------------------------------------
        avg_speed = sum(
            math.hypot(self.robot_states[rid].vx, self.robot_states[rid].vy)
            for rid in active_ids
        ) / num_active

        # ------------------------------------------------------------------
        # 4. Collision detection
        # ------------------------------------------------------------------
        current_pairs: set = set()
        positions = [(rid, self.robot_states[rid].x, self.robot_states[rid].y)
                     for rid in active_ids]

        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                rid_a, xa, ya = positions[i]
                rid_b, xb, yb = positions[j]
                if math.hypot(xa - xb, ya - yb) < COLLISION_RADIUS:
                    pair = (min(rid_a, rid_b), max(rid_a, rid_b))
                    current_pairs.add(pair)

        # Count new collision events (pairs that weren't colliding before)
        new_events = current_pairs - self._prev_collision_pairs
        self._cumulative_collisions += len(new_events)
        for _ in new_events:
            self._collision_events_window.append(now)
        self._prev_collision_pairs = current_pairs

        # ------------------------------------------------------------------
        # 5. Graph-based split detection (BFS connected components)
        # ------------------------------------------------------------------
        num_subgroups, is_split = self._detect_split(active_ids)

        if is_split:
            if self._split_started_at is None:
                self._split_started_at = now
            self.split_timer = now - self._split_started_at
            if self.split_timer >= self.split_recovery_trigger_s and not self._emergency_active:
                self.broadcast_emergency_cohesion(active_ids)
        else:
            self._split_started_at = None
            self.split_timer = 0.0

        if self._emergency_active and now >= self._emergency_restore_at:
            self._restore_emergency_cohesion(active_ids)

        active_wp = self._get_active_waypoint()
        if active_wp is not None:
            near_count = sum(
                1
                for rid in active_ids
                if math.hypot(self.robot_states[rid].x - active_wp[0], self.robot_states[rid].y - active_wp[1])
                <= self.waypoint_reach_radius
            )
            required = max(1, int(math.ceil(self.waypoint_reach_fraction * max(1, num_active))))
            if self.current_waypoint_index < max(0, len(self.waypoints) - 1) and near_count >= required:
                self.current_waypoint_index += 1
                self.waypoints_completed = max(self.waypoints_completed, self.current_waypoint_index)
                active_wp = self._get_active_waypoint()
                if active_wp is not None:
                    self.get_logger().info(
                        f'Advanced to waypoint {self.current_waypoint_index}/{len(self.waypoints)-1}: '
                        f'({active_wp[0]:.2f}, {active_wp[1]:.2f})'
                    )

        if active_wp is not None:
            wp_msg = PointStamped()
            wp_msg.header.stamp = stamp
            wp_msg.header.frame_id = 'map'
            wp_msg.point.x = float(active_wp[0])
            wp_msg.point.y = float(active_wp[1])
            wp_msg.point.z = 0.0
            self.active_waypoint_pub.publish(wp_msg)

        # Goal completion check (computed early so it can be published in state_msg)
        all_reached_goal = False
        at_final_waypoint = self.current_waypoint_index >= max(0, len(self.waypoints) - 1)
        if at_final_waypoint and self.final_waypoint is not None and num_active == self.num_robots:
            gx, gy = self.final_waypoint
            all_reached_goal = all(
                math.hypot(self.robot_states[rid].x - gx, self.robot_states[rid].y - gy) < self.goal_tolerance
                for rid in active_ids
            )
        self.goal_reached = all_reached_goal or self.goal_reached

        # ------------------------------------------------------------------
        # 6. Publish FlockState
        # ------------------------------------------------------------------
        elapsed_time = now - self.start_time
        run_collision_rate = self._cumulative_collisions / elapsed_time if elapsed_time > 0.0 else 0.0
        run_mean_cohesion = self.sum_cohesion / self.total_steps if self.total_steps > 0 else 0.0

        # Keep a sliding cohesion history to produce rolling metrics.
        self._cohesion_window.append((now, cohesion_radius))
        self._prune_rolling_windows(now)

        window_duration = self._rolling_duration(now)
        collision_rate = (
            len(self._collision_events_window) / window_duration if window_duration > 0.0 else 0.0
        )
        mean_cohesion = self._rolling_mean_cohesion()

        state_msg = FlockState()
        state_msg.header.stamp = stamp
        state_msg.header.frame_id = 'map'
        state_msg.centroid.x = cx
        state_msg.centroid.y = cy
        state_msg.centroid.z = 0.0
        state_msg.cohesion_radius    = float(cohesion_radius)
        state_msg.avg_speed          = float(avg_speed)
        state_msg.num_active_robots  = num_active
        state_msg.collision_count    = self._cumulative_collisions
        state_msg.is_split           = is_split
        state_msg.num_subgroups      = num_subgroups
        state_msg.total_time         = float(elapsed_time)
        state_msg.collision_rate     = float(collision_rate)
        state_msg.mean_cohesion      = float(mean_cohesion)
        state_msg.goal_reached       = self.goal_reached
        state_msg.experiment_success = self.experiment_success
        state_msg.completion_reason  = self.completion_reason
        self.state_pub.publish(state_msg)

        # ------------------------------------------------------------------
        # 7. Publish centroid marker
        # ------------------------------------------------------------------
        centroid_msg = PointStamped()
        centroid_msg.header.stamp = stamp
        centroid_msg.header.frame_id = 'map'
        centroid_msg.point.x = cx
        centroid_msg.point.y = cy
        centroid_msg.point.z = 0.05
        self.centroid_pub.publish(centroid_msg)

        # ------------------------------------------------------------------
        # 8. Convex hull marker
        # ------------------------------------------------------------------
        hull_marker = self._build_hull_marker(active_ids, stamp, is_split)
        self.hull_pub.publish(hull_marker)

        # ------------------------------------------------------------------
        # 9. Neighbor link markers
        # ------------------------------------------------------------------
        links_array = self._build_link_markers(active_ids, stamp)
        self.links_pub.publish(links_array)

        # ------------------------------------------------------------------
        # 10. Goal Completion Detection / Success Classification
        # ------------------------------------------------------------------

        if now - self._last_log_time >= self.log_interval_s:
            self._last_log_time = now
            self.get_logger().info(
                f'active={num_active}/{self.num_robots} split={is_split} groups={num_subgroups} '
                f'cohesion={cohesion_radius:.2f}m rolling_collision_rate={collision_rate:.3f}/s '
                f'wp={self.current_waypoint_index}/{max(0, len(self.waypoints)-1)} '
                f'split_t={self.split_timer:.1f}s t={elapsed_time:.1f}s'
            )

        if all_reached_goal:
            reasons = []
            if elapsed_time > self.success_timeout_s:
                reasons.append('timeout_exceeded')
            if run_collision_rate > self.success_max_collision_rate:
                reasons.append('collision_rate_exceeded')
            if run_mean_cohesion > self.success_max_mean_cohesion:
                reasons.append('cohesion_exceeded')

            success = len(reasons) == 0
            reason = 'success' if success else '+'.join(reasons)
            self._finalize_experiment(
                success,
                reason,
                elapsed_time,
                run_collision_rate,
                run_mean_cohesion,
            )
            return

        if elapsed_time >= self.success_timeout_s:
            self._finalize_experiment(
                False,
                'timeout_before_goal',
                elapsed_time,
                run_collision_rate,
                run_mean_cohesion,
            )

    def _prune_rolling_windows(self, now: float) -> None:
        cutoff = now - self.rolling_window_s
        while self._collision_events_window and self._collision_events_window[0] < cutoff:
            self._collision_events_window.popleft()
        while self._cohesion_window and self._cohesion_window[0][0] < cutoff:
            self._cohesion_window.popleft()

    def _rolling_duration(self, now: float) -> float:
        if not self._cohesion_window:
            return self.rolling_window_s
        oldest_ts = self._cohesion_window[0][0]
        return max(1e-3, min(self.rolling_window_s, now - oldest_ts))

    def _rolling_mean_cohesion(self) -> float:
        if not self._cohesion_window:
            return 0.0
        return float(sum(v for _, v in self._cohesion_window) / len(self._cohesion_window))

    def _get_active_waypoint(self):
        if not self.waypoints:
            return None
        idx = max(0, min(self.current_waypoint_index, len(self.waypoints) - 1))
        return self.waypoints[idx]

    def _candidate_boid_node_names(self, rid: int) -> List[str]:
        return [
            f'/robot_{rid}/boid_{rid}',
            f'/robot_{rid}/boid_node',
            f'/boid_{rid}',
            f'/boid_node_{rid}',
            f'/boid_node',
        ]

    def _ros2_param_get_float(self, node_name: str, param_name: str):
        try:
            result = subprocess.run(
                ['ros2', 'param', 'get', node_name, param_name],
                capture_output=True,
                text=True,
                timeout=1.2,
                check=False,
            )
            if result.returncode != 0:
                return None
            text = f'{result.stdout}\n{result.stderr}'
            m = re.search(r'([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', text)
            if m is None:
                return None
            return float(m.group(1))
        except Exception:
            return None

    def _ros2_param_set(self, node_name: str, param_name: str, value: float) -> bool:
        try:
            result = subprocess.run(
                ['ros2', 'param', 'set', node_name, param_name, str(value)],
                capture_output=True,
                text=True,
                timeout=1.2,
                check=False,
            )
            return result.returncode == 0
        except Exception:
            return False

    def _resolve_boid_node_name(self, rid: int) -> str:
        cached = self._boid_node_name_cache.get(rid)
        if cached:
            return cached

        for name in self._candidate_boid_node_names(rid):
            probe = self._ros2_param_get_float(name, 'w_cohesion')
            if probe is not None:
                self._boid_node_name_cache[rid] = name
                return name

        return ''

    def broadcast_emergency_cohesion(self, active_ids: List[int]) -> None:
        """Temporarily enforce high cohesion/low migration when split persists."""
        if self._emergency_active:
            return

        self._emergency_original_weights = {}
        for rid in active_ids:
            node_name = self._resolve_boid_node_name(rid)
            if not node_name:
                continue

            orig_coh = self._ros2_param_get_float(node_name, 'w_cohesion')
            orig_mig = self._ros2_param_get_float(node_name, 'w_migration')
            if orig_coh is None:
                orig_coh = self.default_w_cohesion_restore
            if orig_mig is None:
                orig_mig = self.default_w_migration_restore
            self._emergency_original_weights[rid] = (orig_coh, orig_mig)

            self._ros2_param_set(node_name, 'w_cohesion', self.emergency_w_cohesion)
            self._ros2_param_set(node_name, 'w_migration', self.emergency_w_migration)

        self._emergency_active = True
        self._emergency_restore_at = time.monotonic() + self.split_recovery_duration_s
        self.split_events_count += 1
        self.get_logger().warn(
            f'Split persisted for {self.split_timer:.1f}s. Emergency cohesion broadcast applied '
            f'(event={self.split_events_count}).'
        )

    def _restore_emergency_cohesion(self, active_ids: List[int]) -> None:
        if not self._emergency_active:
            return

        restore_ids = sorted(set(active_ids) | set(self._emergency_original_weights.keys()))
        for rid in restore_ids:
            node_name = self._resolve_boid_node_name(rid)
            if not node_name:
                continue

            orig_coh, orig_mig = self._emergency_original_weights.get(
                rid,
                (self.default_w_cohesion_restore, self.default_w_migration_restore),
            )
            self._ros2_param_set(node_name, 'w_cohesion', orig_coh)
            self._ros2_param_set(node_name, 'w_migration', orig_mig)

        self._emergency_active = False
        self._emergency_restore_at = 0.0
        self._emergency_original_weights = {}
        self.get_logger().info('Restored boid weights after emergency cohesion window.')

    def _finalize_experiment(
        self,
        success: bool,
        reason: str,
        elapsed_time: float,
        collision_rate: float,
        mean_cohesion: float,
    ) -> None:
        if self.experiment_completed:
            return
        self.experiment_completed = True
        self.goal_reached = self.goal_reached or reason == 'success'
        self.experiment_success = bool(success)
        self.completion_reason = reason

        stop_msg = Twist()
        for pub in self.cmd_pubs.values():
            pub.publish(stop_msg)

        status = 'SUCCESS' if success else 'FAILURE'
        self.get_logger().info(
            f'EXPERIMENT COMPLETE: {status} reason={reason} '
            f't={elapsed_time:.2f}s collision_rate={collision_rate:.4f} mean_cohesion={mean_cohesion:.4f}'
        )

        self._log_results_to_csv(success, reason, elapsed_time, collision_rate, mean_cohesion)

        if self.auto_shutdown_on_completion and self._shutdown_timer is None:
            self._shutdown_timer = self.create_timer(1.0, self._shutdown_callback)

    def _log_results_to_csv(self, success, reason, elapsed_time, collision_rate, mean_cohesion):
        file_exists = os.path.isfile(self.csv_filename)
        try:
            with open(self.csv_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                if not file_exists:
                    writer.writerow([
                        'num_robots',
                        'total_time_s',
                        'collision_rate_per_s',
                        'mean_cohesion_radius_m',
                        'waypoints_completed',
                        'split_events_count',
                        'status',
                        'reason',
                        'total_collisions',
                        'success_timeout_s',
                        'success_max_collision_rate',
                        'success_max_mean_cohesion',
                    ])
                writer.writerow([
                    self.num_robots,
                    round(elapsed_time, 3),
                    round(collision_rate, 4),
                    round(mean_cohesion, 4),
                    int(self.waypoints_completed),
                    int(self.split_events_count),
                    'success' if success else 'failure',
                    reason,
                    self._cumulative_collisions,
                    round(self.success_timeout_s, 3),
                    round(self.success_max_collision_rate, 4),
                    round(self.success_max_mean_cohesion, 4),
                ])
            self.get_logger().info(f"Appended configuration and metrics to {self.csv_filename}.")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")

    def _shutdown_callback(self):
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
            self._shutdown_timer = None
        self.get_logger().info("Flushing complete. Triggering safe system tear-down.")
        self._safe_shutdown_context()

    # ====================================================================
    # Split detection via BFS
    # ====================================================================

    def _detect_split(self, active_ids: List[int]) -> Tuple[int, bool]:
        """
        Build an adjacency graph where robots within neighbour_radius are
        connected.  Count connected components via BFS.

        Returns (num_components, is_split).
        """
        if len(active_ids) <= 1:
            return (len(active_ids), False)

        # Build adjacency list
        adj: Dict[int, List[int]] = {rid: [] for rid in active_ids}
        for i in range(len(active_ids)):
            for j in range(i + 1, len(active_ids)):
                rid_a = active_ids[i]
                rid_b = active_ids[j]
                sa = self.robot_states[rid_a]
                sb = self.robot_states[rid_b]
                if math.hypot(sa.x - sb.x, sa.y - sb.y) <= self.neighbour_r:
                    adj[rid_a].append(rid_b)
                    adj[rid_b].append(rid_a)

        # BFS to count components
        visited: set = set()
        num_components = 0
        for start in active_ids:
            if start in visited:
                continue
            num_components += 1
            queue = deque([start])
            while queue:
                node = queue.popleft()
                if node in visited:
                    continue
                visited.add(node)
                for neighbour in adj[node]:
                    if neighbour not in visited:
                        queue.append(neighbour)

        return (num_components, num_components > 1)

    # ====================================================================
    # Convex hull (Graham scan)
    # ====================================================================

    def _convex_hull(
        self, points: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """
        Graham scan convex hull.  Returns vertices in CCW order.
        Handles degenerate cases (< 3 unique points) gracefully.
        """
        # Deduplicate
        pts = list(set((round(x, 6), round(y, 6)) for x, y in points))

        if len(pts) < 2:
            return pts
        if len(pts) == 2:
            return pts

        # Find bottom-most (then left-most) point as pivot
        pivot = min(pts, key=lambda p: (p[1], p[0]))

        def polar_angle(p):
            dx = p[0] - pivot[0]
            dy = p[1] - pivot[1]
            return math.atan2(dy, dx)

        def dist_sq(p):
            dx = p[0] - pivot[0]
            dy = p[1] - pivot[1]
            return dx * dx + dy * dy

        # Sort by polar angle, break ties by distance
        sorted_pts = sorted(pts, key=lambda p: (polar_angle(p), dist_sq(p)))

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        hull = []
        for p in sorted_pts:
            while len(hull) >= 2 and cross(hull[-2], hull[-1], p) <= 0:
                hull.pop()
            hull.append(p)

        return hull

    # ====================================================================
    # RViz marker builders
    # ====================================================================

    def _build_hull_marker(self, active_ids: List[int], stamp, is_split: bool) -> Marker:
        """Build a LINE_STRIP marker tracing the convex hull of the swarm."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'
        marker.ns = 'flock_hull'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05   # line width (m)
        if is_split:
            marker.color.r = 0.9
            marker.color.g = 0.2
            marker.color.b = 0.2
        else:
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.2
        marker.color.a = 0.8
        marker.pose.orientation.w = 1.0

        pts_2d = [
            (self.robot_states[rid].x, self.robot_states[rid].y)
            for rid in active_ids
        ]
        hull = self._convex_hull(pts_2d)

        if len(hull) < 2:
            # Just a dot or single point — draw a tiny loop
            for hx, hy in hull:
                p = Point()
                p.x, p.y, p.z = hx, hy, 0.05
                marker.points.append(p)
        else:
            # Close the polygon
            for hx, hy in hull:
                p = Point()
                p.x, p.y, p.z = hx, hy, 0.05
                marker.points.append(p)
            # Close by repeating first point
            p = Point()
            p.x, p.y, p.z = hull[0][0], hull[0][1], 0.05
            marker.points.append(p)

        return marker

    def _build_link_markers(
        self, active_ids: List[int], stamp
    ) -> MarkerArray:
        """
        Build a MarkerArray of thin LINE_LIST markers connecting every pair
        of robots within neighbour_radius.
        """
        array = MarkerArray()

        # First, delete old markers to avoid ghost lines
        delete_all = Marker()
        delete_all.header.stamp = stamp
        delete_all.header.frame_id = 'map'
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)

        marker_id = 1
        for i in range(len(active_ids)):
            for j in range(i + 1, len(active_ids)):
                rid_a = active_ids[i]
                rid_b = active_ids[j]
                sa = self.robot_states[rid_a]
                sb = self.robot_states[rid_b]
                dist = math.hypot(sa.x - sb.x, sa.y - sb.y)

                if dist > self.neighbour_r:
                    continue

                link = Marker()
                link.header.stamp = stamp
                link.header.frame_id = 'map'
                link.ns = 'flock_links'
                link.id = marker_id
                link.type = Marker.LINE_LIST
                link.action = Marker.ADD
                link.scale.x = 0.02  # thin line
                link.color.r = 0.3
                link.color.g = 0.6
                link.color.b = 1.0
                link.color.a = 0.5
                link.pose.orientation.w = 1.0

                pa = Point()
                pa.x, pa.y, pa.z = sa.x, sa.y, 0.05
                pb = Point()
                pb.x, pb.y, pb.z = sb.x, sb.y, 0.05
                link.points.extend([pa, pb])

                array.markers.append(link)
                marker_id += 1

        return array


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = FlockMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node._safe_shutdown_context()


if __name__ == '__main__':
    main()
