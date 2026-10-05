from __future__ import annotations
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
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
        self.timer = self.create_timer(0.5, self._tick)
        self._write('RUNNING', {})

    def _gt_callback(self, msg): self.gt = pose2d_from_pose(msg.pose.pose)
    def _loc_callback(self, msg): self.localized = pose2d_from_pose(msg.pose)
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
        elapsed = time.monotonic() - self.started
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
        metrics = {'elapsed_sec': round(elapsed, 2), 'occupied_cells': self.map_counts[0],
                   'free_cells': self.map_counts[1], 'path_poses': self.path_size,
                   'localization_error_m': None if loc_error is None else round(loc_error, 3),
                   'goal_error_m': None if goal_error is None else round(goal_error, 3)}
        passed = (self.goal_sent and self.map_counts[0] >= 150 and self.map_counts[1] >= 2500 and
                  self.path_size >= 3 and loc_error is not None and loc_error < 0.65 and
                  goal_error is not None and goal_error < 0.45)
        if passed:
            self._write('PASS', metrics)
        elif elapsed > 45.0:
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
