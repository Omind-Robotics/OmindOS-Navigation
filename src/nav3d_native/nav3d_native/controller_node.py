from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

from .algorithms import pure_pursuit
from .ros_utils import pose2d_from_pose

PATH_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)


class ControllerNode(Node):
    def __init__(self):
        super().__init__('nav3d_controller')
        self.pose = None
        self.path = []
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/localization/pose', self._pose_callback, 10)
        self.create_subscription(Path, '/plan', self._path_callback, PATH_QOS)
        self.timer = self.create_timer(0.05, self._control)

    def _pose_callback(self, msg): self.pose = pose2d_from_pose(msg.pose)

    def _path_callback(self, msg): self.path = [
        (p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _control(self):
        cmd = Twist()
        if self.pose is not None and self.path:
            cmd.linear.x, cmd.angular.z, _ = pure_pursuit(self.pose, self.path)
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
