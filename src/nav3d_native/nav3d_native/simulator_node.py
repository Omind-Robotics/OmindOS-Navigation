from __future__ import annotations
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .algorithms import Pose2D, clamp, inverse_transform, normalize_angle, transform_pose
from .ros_utils import quaternion_from_yaw


class NativeSimulator(Node):
    def __init__(self):
        super().__init__('nav3d_simulator')
        self.declare_parameter('mapping_phase_seconds', 5.0)
        self.declare_parameter('scan_rate', 10.0)
        self.mapping_seconds = float(self.get_parameter('mapping_phase_seconds').value)
        self.scan_rate = float(self.get_parameter('scan_rate').value)
        self.pose = Pose2D(-3.5, -2.5, 0.0)
        self.command = Twist()
        self.started = time.monotonic()
        self.last_update = self.started
        self.mapping_poses = self._make_mapping_poses()
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.gt_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self._command_callback, 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        static = TransformStamped()
        static.header.stamp = self.get_clock().now().to_msg()
        static.header.frame_id = 'base_link'
        static.child_frame_id = 'base_scan'
        static.transform.rotation.w = 1.0
        self.static_tf.sendTransform(static)
        self.timer = self.create_timer(1.0 / self.scan_rate, self._tick)
        self.get_logger().info('Deterministic native simulator started')

    @staticmethod
    def _make_mapping_poses():
        poses = []
        ys = [-3.1, -1.55, 0.0, 1.55, 3.1]
        for row, y in enumerate(ys):
            xs = [-4.2 + i * 0.7 for i in range(13)]
            if row % 2:
                xs.reverse()
            yaw = 0.0 if row % 2 == 0 else math.pi
            poses.extend(Pose2D(x, y, yaw) for x in xs)
        return poses

    @staticmethod
    def _occupied(x: float, y: float) -> bool:
        if x <= -5.0 or x >= 5.0 or y <= -4.0 or y >= 4.0:
            return True
        rectangles = [(-1.0, 1.0, -0.5, 0.5), (1.8, 2.4, 1.0, 3.25), (-3.25, -2.55, 0.8, 2.8)]
        return any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in rectangles)

    def _command_callback(self, message: Twist):
        self.command = message

    def _raycast(self, angle: float, max_range: float) -> float:
        distance = 0.08
        while distance < max_range:
            x = self.pose.x + distance * math.cos(angle)
            y = self.pose.y + distance * math.sin(angle)
            if self._occupied(x, y):
                return distance
            distance += 0.04
        return max_range

    def _publish_odom(self, publisher, frame: str, child: str, pose: Pose2D, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.child_frame_id = child
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.orientation = quaternion_from_yaw(pose.yaw)
        publisher.publish(msg)

    def _tick(self):
        now_mono = time.monotonic()
        elapsed = now_mono - self.started
        dt = min(0.2, now_mono - self.last_update)
        self.last_update = now_mono
        if elapsed < self.mapping_seconds:
            fraction = elapsed / max(self.mapping_seconds, 0.1)
            index = min(len(self.mapping_poses) - 1, int(fraction * len(self.mapping_poses)))
            self.pose = self.mapping_poses[index]
        else:
            if elapsed - self.mapping_seconds < 0.25:
                self.pose = Pose2D(-3.5, -2.5, 0.0)
            else:
                linear = clamp(self.command.linear.x, -0.7, 0.7)
                angular = clamp(self.command.angular.z, -1.5, 1.5)
                next_yaw = normalize_angle(self.pose.yaw + angular * dt)
                nx = self.pose.x + linear * math.cos(next_yaw) * dt
                ny = self.pose.y + linear * math.sin(next_yaw) * dt
                if not self._occupied(nx, ny):
                    self.pose.x, self.pose.y = nx, ny
                self.pose.yaw = next_yaw
        stamp = self.get_clock().now().to_msg()
        self._publish_odom(self.gt_pub, 'map', 'base_link_gt', self.pose, stamp)
        # A fixed map->odom transform models a realistic drifting local frame.
        map_to_odom = Pose2D(-0.35, 0.25, -0.08)
        biased = transform_pose(inverse_transform(map_to_odom), self.pose)
        self._publish_odom(self.odom_pub, 'odom', 'base_link', biased, stamp)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = biased.x
        transform.transform.translation.y = biased.y
        transform.transform.rotation = quaternion_from_yaw(biased.yaw)
        self.tf.sendTransform(transform)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_scan'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 90.0
        scan.range_min = 0.08
        scan.range_max = 8.0
        scan.ranges = [
            self._raycast(
                self.pose.yaw + scan.angle_min + i * scan.angle_increment,
                scan.range_max)
            for i in range(181)]
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = NativeSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
