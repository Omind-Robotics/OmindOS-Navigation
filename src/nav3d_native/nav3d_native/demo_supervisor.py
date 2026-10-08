from __future__ import annotations
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from std_msgs.msg import String

from .ros_utils import pose2d_from_pose

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)
PATH_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)


class DemoSupervisor(Node):
    def __init__(self):
        super().__init__('nav3d_demo_supervisor')
        self.declare_parameter('mapping_phase_seconds', 5.0)
        self.declare_parameter('result_file', '/tmp/nav3d_demo_result.json')
        self.declare_parameter('timeout_seconds', 45.0)
        self.declare_parameter('require_stopped', False)
        self.timeout = float(self.get_parameter('timeout_seconds').value)
        self.require_stopped = bool(self.get_parameter('require_stopped').value)
        self.last_command_at = None
        self.stopped_since = None
        self.motion_seen = False
        self.last_gt_at = None
        self.last_loc_at = None
        self.mapping_seconds = float(self.get_parameter('mapping_phase_seconds').value)
        self.result_file = Path(str(self.get_parameter('result_file').value))
        self.started = time.monotonic()
        self.goal_sent = False
        self.goal = (3.5, 2.5)
        self.gt = self.localized = None
        self.map_counts = (0, 0)
        self.path_size = 0
        self.result_pub = self.create_publisher(String, '/nav3d/demo_result', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._gt_callback, 10)
        self.create_subscription(PoseStamped, '/localization/pose', self._loc_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, MAP_QOS)
        self.create_subscription(NavPath, '/plan', self._path_callback, PATH_QOS)
        self.create_subscription(Twist, '/cmd_vel', self._command_callback, 1)
        self.timer = self.create_timer(0.5, self._tick)
        self._write('RUNNING', {})

    def _gt_callback(self, msg):
        self.gt = pose2d_from_pose(msg.pose.pose)
        self.last_gt_at = time.monotonic()

    def _loc_callback(self, msg):
        self.localized = pose2d_from_pose(msg.pose)
        self.last_loc_at = time.monotonic()

    def _command_callback(self, msg):
        now = time.monotonic()
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        stopped = all(math.isfinite(v) and abs(v) < 1e-4 for v in values)
        if not stopped:
            self.motion_seen = True
            self.stopped_since = None
        elif (self.stopped_since is None or self.last_command_at is None
              or now - self.last_command_at > 0.3):
            self.stopped_since = now
        self.last_command_at = now

    def _map_callback(self, msg): self.map_counts = (
        sum(v >= 65 for v in msg.data), sum(v == 0 for v in msg.data))

    def _path_callback(self, msg): self.path_size = max(self.path_size, len(msg.poses))

    def _write(self, status, metrics):
        payload = {'status': status, **metrics}
        self.result_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.result_file.with_suffix('.tmp')
        tmp.write_text(json.dumps(payload, indent=2), encoding='utf-8')
        tmp.replace(self.result_file)
        message = String()
        message.data = json.dumps(payload)
        self.result_pub.publish(message)

    def _tick(self):
        now = time.monotonic()
        elapsed = now - self.started
        if elapsed >= self.mapping_seconds + 1.0:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x, goal.pose.position.y = self.goal
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.goal_sent = True
        loc_error = None if self.gt is None or self.localized is None else math.hypot(
            self.gt.x - self.localized.x, self.gt.y - self.localized.y)
        goal_error = None if self.gt is None else math.hypot(
            self.gt.x - self.goal[0], self.gt.y - self.goal[1])
        stopped_sec = 0.0 if self.stopped_since is None else now - self.stopped_since
        stopped = (self.last_command_at is not None and now - self.last_command_at <= 0.3
                   and stopped_sec >= 1.0 and self.motion_seen)
        fresh = (self.last_gt_at is not None and now - self.last_gt_at <= 0.5
                 and self.last_loc_at is not None and now - self.last_loc_at <= 0.75)
        metrics = {'elapsed_sec': round(elapsed, 2), 'occupied_cells': self.map_counts[0],
                   'free_cells': self.map_counts[1], 'path_poses': self.path_size,
                   'localization_error_m': None if loc_error is None else round(loc_error, 3),
                   'goal_error_m': None if goal_error is None else round(goal_error, 3),
                   'motion_seen': self.motion_seen, 'stopped_sec': round(stopped_sec, 2)}
        passed = (self.goal_sent and self.map_counts[0] >= 150 and self.map_counts[1] >= 2500 and
                  self.path_size >= 3 and loc_error is not None and loc_error < 0.65 and
                  goal_error is not None and goal_error < 0.45 and
                  (not self.require_stopped or (stopped and fresh)))
        if passed:
            self._write('PASS', metrics)
        elif elapsed > self.timeout:
            self._write('FAIL', metrics)
        else:
            self._write('RUNNING', metrics)


def main(args=None):
    rclpy.init(args=args)
    node = DemoSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
