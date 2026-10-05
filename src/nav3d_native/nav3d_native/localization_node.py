from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from .algorithms import Pose2D, normalize_angle, scan_match, transform_between, transform_pose
from .ros_utils import pose2d_from_pose, quaternion_from_yaw

MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('nav3d_localizer')
        self.map = None
        self.odom = None
        self.correction = Pose2D(0.0, 0.0, 0.0)
        self.initialized = False
        self.publisher = self.create_publisher(PoseStamped, '/localization/pose', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, MAP_QOS)
        self.create_subscription(Odometry, '/wheel/odom', self._odom_callback, 5)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 1)

    def _map_callback(self, msg):
        self.map = msg

    def _odom_callback(self, msg):
        self.odom = pose2d_from_pose(msg.pose.pose)

    def _scan_callback(self, scan):
        if self.map is None or self.odom is None:
            return
        predicted = transform_pose(self.correction, self.odom)
        matched, score = scan_match(
            self.map.data, self.map.info.width, self.map.info.height, self.map.info.resolution,
            self.map.info.origin.position.x, self.map.info.origin.position.y, predicted,
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_max,
            xy_window=0.2 if self.initialized else 0.6,
            yaw_window=0.08 if self.initialized else 0.16)
        if score > -1.0e20:
            target = transform_between(matched, self.odom)
            if not self.initialized:
                self.correction = target
                self.initialized = True
            else:
                alpha = 0.2
                self.correction = Pose2D(
                    (1.0 - alpha) * self.correction.x + alpha * target.x,
                    (1.0 - alpha) * self.correction.y + alpha * target.y,
                    normalize_angle(
                        self.correction.yaw + alpha * normalize_angle(
                            target.yaw - self.correction.yaw)))
            estimate = transform_pose(self.correction, self.odom)
            self._publish(estimate, scan.header.stamp)

    def _publish(self, pose, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.orientation = quaternion_from_yaw(pose.yaw)
        self.publisher.publish(msg)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = self.correction.x
        transform.transform.translation.y = self.correction.y
        transform.transform.rotation = quaternion_from_yaw(self.correction.yaw)
        self.tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
