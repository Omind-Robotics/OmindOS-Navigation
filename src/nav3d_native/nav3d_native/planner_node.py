from __future__ import annotations
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

from .algorithms import astar, simplify_path
from .ros_utils import quaternion_from_yaw

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)
PATH_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)


class PlannerNode(Node):
    def __init__(self):
        super().__init__('nav3d_planner')
        self.map = None
        self.pose = None
        self.goal = None
        self.publisher = self.create_publisher(Path, '/plan', PATH_QOS)
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, MAP_QOS)
        self.create_subscription(PoseStamped, '/localization/pose', self._pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_callback, 10)
        self.timer = self.create_timer(1.0, self._plan)

    def _map_callback(self, msg): self.map = msg
    def _pose_callback(self, msg): self.pose = msg
    def _goal_callback(self, msg): self.goal = msg

    def _grid(self, x, y):
        return (int(math.floor((x - self.map.info.origin.position.x) / self.map.info.resolution)),
                int(math.floor((y - self.map.info.origin.position.y) / self.map.info.resolution)))

    def _world(self, gx, gy):
        return (self.map.info.origin.position.x + (gx + 0.5) * self.map.info.resolution,
                self.map.info.origin.position.y + (gy + 0.5) * self.map.info.resolution)

    def _plan(self):
        if self.map is None or self.pose is None or self.goal is None:
            return
        cells = astar(self.map.data, self.map.info.width, self.map.info.height,
                      self._grid(self.pose.pose.position.x, self.pose.pose.position.y),
                      self._grid(self.goal.pose.position.x, self.goal.pose.position.y), 6)
        if not cells:
            self.get_logger().warning('No collision-free route found', throttle_duration_sec=3.0)
            return
        points = simplify_path([self._world(x, y) for x, y in cells], 0.3)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if i + 1 < len(points):
                yaw = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
                pose.pose.orientation = quaternion_from_yaw(yaw)
            else:
                pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
