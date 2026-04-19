#!/usr/bin/env python3
# FILE: swarm_flocking/physics_node.py
"""
PhysicsNode — Lightweight 2D Kinematics Engine.

Responsibilities:
  • Replaces Gazebo by implementing a strict zero-order hold decoupled simulation.
  • Subscribes to all /robot_i/cmd_vel streams.
  • Integrates dt=0.1s kinematic bounds securely.
  • Synthesizes and publishes strict /robot_i/odom arrays and blank /robot_i/scan.
"""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from swarm_flocking.utils.reynolds import normalize_angle, clamp

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

class PhysicsNode(Node):
    def __init__(self):
        super().__init__('physics_node')

        self.declare_parameter('num_robots', 6)
        self.declare_parameter('max_linear_vel', 0.22)
        self.declare_parameter('max_angular_vel', 2.8)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('scan_num_rays', 360)
        self.declare_parameter('scan_range_min', 0.12)
        self.declare_parameter('scan_range_max', 3.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter(
            'spawn_coords',
            [0.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )  # flat [x0,y0,x1,y1,...]
        self.declare_parameter(
            'obstacle_segments',
            [0.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )

        self.num_robots = int(self.get_parameter('num_robots').value)
        self.max_lin = float(self.get_parameter('max_linear_vel').value)
        self.max_ang = float(self.get_parameter('max_angular_vel').value)
        self.dt = max(0.01, float(self.get_parameter('dt').value))
        self.scan_num_rays = max(8, int(self.get_parameter('scan_num_rays').value))
        self.scan_range_min = max(0.01, float(self.get_parameter('scan_range_min').value))
        self.scan_range_max = max(self.scan_range_min + 0.1, float(self.get_parameter('scan_range_max').value))
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.spawn_offsets = self._parse_spawn_offsets(self.get_parameter('spawn_coords').value)
        self.obstacle_segments = self._parse_or_default_obstacles(
            self.get_parameter('obstacle_segments').value
        )

        # Internal kinematic state: x, y, theta, vx, omega
        self.state = {
            i: {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'vx': 0.0, 'omega': 0.0}
            for i in range(self.num_robots)
        }

        self.odom_pubs = {}
        self.scan_pubs = {}
        self.tf_pub = None

        for i in range(self.num_robots):
            self.create_subscription(
                Twist,
                f'/robot_{i}/cmd_vel',
                lambda msg, rid=i: self._cmd_cb(rid, msg),
                STATE_QOS
            )
            self.odom_pubs[i] = self.create_publisher(Odometry, f'/robot_{i}/odom', SENSOR_QOS)
            self.scan_pubs[i] = self.create_publisher(LaserScan, f'/robot_{i}/scan', SENSOR_QOS)

        if self.publish_tf:
            self.tf_pub = self.create_publisher(TFMessage, '/tf', STATE_QOS)

        self.timer = self.create_timer(self.dt, self._physics_step)

        self.get_logger().info(
            f'PhysicsNode booted for {self.num_robots} robots using headless 2D kinematics '
            f'(dt={self.dt:.3f}s, rays={self.scan_num_rays}, tf={self.publish_tf}).'
        )

    def _parse_spawn_offsets(self, flat_coords) -> Dict[int, Tuple[float, float]]:
        flat = [float(v) for v in list(flat_coords)] if flat_coords else []
        if len(flat) % 2 != 0:
            flat = flat[:-1]

        offsets: Dict[int, Tuple[float, float]] = {}
        for i in range(self.num_robots):
            idx = 2 * i
            if idx + 1 < len(flat):
                offsets[i] = (flat[idx], flat[idx + 1])
            else:
                offsets[i] = (0.0, 0.0)
        return offsets

    def _parse_or_default_obstacles(self, flat_segments) -> List[Tuple[float, float, float, float]]:
        if flat_segments:
            raw = [float(v) for v in list(flat_segments)]
            if len(raw) % 4 != 0:
                raw = raw[: len(raw) - (len(raw) % 4)]
            if raw:
                return [
                    (raw[k], raw[k + 1], raw[k + 2], raw[k + 3])
                    for k in range(0, len(raw), 4)
                ]

        # Default headless obstacle geometry approximating obstacle_course.world.
        return [
            # Outer bounds (30 x 15)
            (0.0, 0.0, 30.0, 0.0),
            (30.0, 0.0, 30.0, 15.0),
            (30.0, 15.0, 0.0, 15.0),
            (0.0, 15.0, 0.0, 0.0),
            # Bottleneck at x=8.5 with central gap
            (8.5, 0.0, 8.5, 6.75),
            (8.5, 8.25, 8.5, 15.0),
            # Partial divider
            (14.0, 0.0, 14.0, 5.0),
            # Maze-inspired segments
            (19.0, 4.0, 23.0, 4.0),
            (21.0, 11.0, 25.0, 11.0),
            (24.0, 5.5, 24.0, 9.5),
            (26.5, 2.5, 26.5, 7.5),
            (24.5, 10.5, 27.5, 10.5),
        ]

    def _ray_segment_intersection(
        self,
        ox: float,
        oy: float,
        dx: float,
        dy: float,
        segment: Tuple[float, float, float, float],
    ) -> Optional[float]:
        x1, y1, x2, y2 = segment
        sx = x2 - x1
        sy = y2 - y1
        denom = dx * sy - dy * sx
        if abs(denom) < 1e-9:
            return None

        rx = x1 - ox
        ry = y1 - oy
        t = (rx * sy - ry * sx) / denom
        u = (rx * dy - ry * dx) / denom
        if t >= 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    def _scan_from_world(self, world_x: float, world_y: float, theta: float) -> List[float]:
        ranges: List[float] = []
        angle_min = -math.pi
        angle_inc = (2.0 * math.pi) / float(self.scan_num_rays)

        for i in range(self.scan_num_rays):
            ray_angle = theta + angle_min + i * angle_inc
            dx = math.cos(ray_angle)
            dy = math.sin(ray_angle)

            best = self.scan_range_max
            for segment in self.obstacle_segments:
                dist = self._ray_segment_intersection(world_x, world_y, dx, dy, segment)
                if dist is not None and self.scan_range_min <= dist < best:
                    best = dist

            if best >= self.scan_range_max:
                ranges.append(float('inf'))
            else:
                ranges.append(float(best))

        return ranges

    def _cmd_cb(self, rid: int, msg: Twist) -> None:
        """Zero-order hold caching of incoming commands."""
        # Safety clamping at the physics layer
        self.state[rid]['vx'] = clamp(msg.linear.x, -self.max_lin, self.max_lin)
        self.state[rid]['omega'] = clamp(msg.angular.z, -self.max_ang, self.max_ang)

    def _physics_step(self) -> None:
        """Synchronous deterministic integration loop."""
        stamp = self.get_clock().now().to_msg()
        tf_transforms = []

        for i in range(self.num_robots):
            s = self.state[i]

            # 1. Closed-loop Kinematic Expansion
            s['x'] += s['vx'] * math.cos(s['theta']) * self.dt
            s['y'] += s['vx'] * math.sin(s['theta']) * self.dt
            s['theta'] = normalize_angle(s['theta'] + s['omega'] * self.dt)

            # 2. Publish Standard /odom
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = f'robot_{i}/base_footprint'

            odom.pose.pose.position.x = float(s['x'])
            odom.pose.pose.position.y = float(s['y'])

            cy = math.cos(s['theta'] * 0.5)
            sy = math.sin(s['theta'] * 0.5)
            odom.pose.pose.orientation.w = float(cy)
            odom.pose.pose.orientation.z = float(sy)

            odom.twist.twist.linear.x = float(s['vx'])
            odom.twist.twist.angular.z = float(s['omega'])

            self.odom_pubs[i].publish(odom)

            # 3. Publish synthetic /scan against obstacle geometry in world frame.
            offset_x, offset_y = self.spawn_offsets.get(i, (0.0, 0.0))
            world_x = s['x'] + offset_x
            world_y = s['y'] + offset_y
            ranges = self._scan_from_world(world_x, world_y, s['theta'])

            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = f'robot_{i}/base_scan'
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = (2.0 * math.pi) / float(self.scan_num_rays)
            scan.range_min = self.scan_range_min
            scan.range_max = self.scan_range_max
            scan.ranges = ranges
            self.scan_pubs[i].publish(scan)

            if self.publish_tf:
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = 'odom'
                tf.child_frame_id = f'robot_{i}/base_footprint'
                tf.transform.translation.x = float(s['x'])
                tf.transform.translation.y = float(s['y'])
                tf.transform.translation.z = 0.0
                tf.transform.rotation.w = float(cy)
                tf.transform.rotation.z = float(sy)
                tf_transforms.append(tf)

        if self.publish_tf and self.tf_pub is not None and tf_transforms:
            tf_msg = TFMessage()
            tf_msg.transforms = tf_transforms
            self.tf_pub.publish(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsNode()
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
