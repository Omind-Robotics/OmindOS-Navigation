"""Simulation-only ROS 2 adapter; intentionally imports no vendor SDK or CAN API."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .platform_guard import PlatformGuard, TIMEOUTS, load_profile, stamp_is_fresh


ROOT = '/omindos_preview'
PATH_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def valid_pose(pose):
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    return (all(math.isfinite(v) for v in values)
            and abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.0) < 0.05)


class PlatformAdapter(Node):
    def __init__(self):
        super().__init__('platform_adapter')
        self.declare_parameter('platform', 'wheeled_diff')
        self.declare_parameter('profile_file', '')
        self.declare_parameter('enable_preview_motion', False)
        profile = load_profile(str(self.get_parameter('platform').value),
                               str(self.get_parameter('profile_file').value) or None)
        self.guard = PlatformGuard(profile, bool(
            self.get_parameter('enable_preview_motion').value))
        self.publisher = self.create_publisher(Twist, ROOT + '/robot/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, ROOT + '/platform_status', 10)
        self.create_subscription(Twist, ROOT + '/navigation/cmd_vel', self._command, 1)
        self.create_subscription(PoseStamped, ROOT + '/localization/pose', self._pose, 1)
        self.create_subscription(Odometry, ROOT + '/odom', self._odom, 1)
        self.create_subscription(Path, ROOT + '/plan', self._plan, PATH_QOS)
        self.create_subscription(Bool, ROOT + '/emergency_stop', self._estop, 1)
        self.timer = self.create_timer(0.02, self._tick)
        self.get_logger().info(f'PREVIEW ONLY: {profile.name}; no hardware driver loaded')

    def _observe(self, header, source, frame, valid):
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = header.frame_id == frame and stamp_is_fresh(stamp, now, TIMEOUTS[source])
        # Count transport delay too; a late message must not earn a fresh full timeout.
        receipt = time.monotonic() - (now - stamp) if fresh else time.monotonic()
        self.guard.observe(source, fresh and valid, receipt)

    def _command(self, msg):
        self.guard.receive_command((msg.linear.x, msg.linear.y, msg.linear.z,
                                    msg.angular.x, msg.angular.y, msg.angular.z),
                                   time.monotonic())

    def _pose(self, msg):
        self._observe(msg.header, 'pose', 'map', valid_pose(msg.pose))

    def _odom(self, msg):
        self._observe(msg.header, 'odom', 'odom', msg.child_frame_id == 'base_link'
                      and valid_pose(msg.pose.pose))

    def _plan(self, msg):
        valid = bool(msg.poses) and all(
            p.header.frame_id == 'map' and valid_pose(p.pose) for p in msg.poses)
        self._observe(msg.header, 'plan', 'map', valid)

    def _estop(self, msg):
        self.guard.emergency_stop(msg.data)
        self._tick()

    def _tick(self):
        status, values = self.guard.evaluate(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = values[:3]
        msg.angular.x, msg.angular.y, msg.angular.z = values[3:]
        self.publisher.publish(msg)
        info = String()
        info.data = json.dumps({'platform': self.guard.profile.name, 'status': status,
                                'preview_only': True, 'output': list(values)})
        self.status_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlatformAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
