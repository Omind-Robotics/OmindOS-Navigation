#!/usr/bin/env python3
"""Kinematic base simulator ported from the original ROS 1 Nav_3d stack."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OfflineBaseSim(Node):
    """Publish a simulated base pose without contacting robot hardware."""

    def __init__(self):
        super().__init__('offline_base_sim')

        self.map_frame = self.parameter('map_frame', 'map')
        self.base_frame = self.parameter('base_frame', 'base_link')
        self.update_rate = self.parameter('update_rate', 50.0)
        self.cmd_timeout = self.parameter('cmd_timeout', 0.35)
        self.max_linear_velocity = self.parameter('max_linear_velocity', 0.9)
        self.max_angular_velocity = self.parameter('max_angular_velocity', 1.4)
        self.motion_mode = self.parameter('motion_mode', 'path')
        self.path_topic = self.parameter('path_topic', '/path_smooth')
        self.cmd_vel_topic = self.parameter('cmd_vel_topic', '/sim/cmd_vel')
        self.path_following_speed = self.parameter(
            'path_following_speed', 0.35)
        self.align_at_path_end = self.parameter('align_at_path_end', True)
        self.follow_path_height = self.parameter('follow_path_height', True)
        self.trace_spacing = self.parameter('trace_spacing', 0.05)
        self.initialpose_uses_z = self.parameter('initialpose_uses_z', False)
        self.enable_map_leveling = self.parameter('enable_map_leveling', False)
        self.initial_pose_is_source_frame = self.parameter(
            'initial_pose_is_source_frame', True)
        self.level_normal = (
            self.parameter('level_normal_x', 0.0),
            self.parameter('level_normal_y', 0.0),
            self.parameter('level_normal_z', 1.0),
        )
        self.level_pivot = (
            self.parameter('level_pivot_x', 0.0),
            self.parameter('level_pivot_y', 0.0),
            self.parameter('level_pivot_z', 0.0),
        )
        if self.motion_mode not in ('path', 'cmd_vel'):
            raise ValueError("motion_mode must be 'path' or 'cmd_vel'")
        self.level_rotation = self.make_level_rotation()

        self.x = self.parameter('initial_x', 0.0)
        self.y = self.parameter('initial_y', 0.0)
        self.z = self.parameter('initial_z', 0.0)
        self.yaw = self.parameter('initial_yaw', 0.0)
        if self.enable_map_leveling and self.initial_pose_is_source_frame:
            self.x, self.y, self.z = self.level_point((self.x, self.y, self.z))
            self.yaw = self.level_yaw(self.yaw)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_command_time = None
        self.last_update_time = self.get_clock().now()
        self.reference_path = []
        self.reference_segment = 0
        self.executed_path = Path()
        self.executed_path.header.frame_id = self.map_frame
        self.last_trace_position = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        trace_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_publisher = self.create_publisher(
            Path, '/simulated_path', trace_qos)
        self.error_publisher = self.create_publisher(
            Float32MultiArray, '/tracking_error', 10)
        self.create_subscription(
            Twist, self.cmd_vel_topic, self.command_callback, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 1)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose', self.initial_pose_callback, 1)

        self.create_timer(1.0 / self.update_rate, self.update)
        self.get_logger().info(
            'Offline base simulator ready in %s mode; command topic is %s. '
            'It never contacts robot hardware.' %
            (self.motion_mode, self.cmd_vel_topic))

    def parameter(self, name, default):
        return self.declare_parameter(name, default).value

    def make_level_rotation(self):
        if not self.enable_map_leveling:
            return ((1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0))

        nx, ny, nz = self.level_normal
        length = math.sqrt(nx * nx + ny * ny + nz * nz)
        if length < 1e-6:
            raise ValueError(
                'Map leveling requested, but level_normal is zero')
        nx, ny, nz = nx / length, ny / length, nz / length

        qx, qy, qz, qw = ny, -nx, 0.0, 1.0 + nz
        q_length = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if q_length < 1e-6:
            qx, qy, qz, qw = 1.0, 0.0, 0.0, 0.0
        else:
            qx, qy, qz, qw = (qx / q_length, qy / q_length,
                              qz / q_length, qw / q_length)

        return (
            (1.0 - 2.0 * (qy * qy + qz * qz),
             2.0 * (qx * qy - qz * qw),
             2.0 * (qx * qz + qy * qw)),
            (2.0 * (qx * qy + qz * qw),
             1.0 - 2.0 * (qx * qx + qz * qz),
             2.0 * (qy * qz - qx * qw)),
            (2.0 * (qx * qz - qy * qw),
             2.0 * (qy * qz + qx * qw),
             1.0 - 2.0 * (qx * qx + qy * qy)),
        )

    def level_vector(self, vector):
        return tuple(sum(self.level_rotation[row][column] * vector[column]
                         for column in range(3)) for row in range(3))

    def level_point(self, point):
        relative = tuple(
            point[index] - self.level_pivot[index] for index in range(3))
        rotated = self.level_vector(relative)
        return tuple(
            rotated[index] + self.level_pivot[index] for index in range(3))

    def level_yaw(self, yaw):
        direction = self.level_vector((math.cos(yaw), math.sin(yaw), 0.0))
        return math.atan2(direction[1], direction[0])

    def command_callback(self, command):
        self.linear_velocity = clamp(
            command.linear.x,
            -self.max_linear_velocity, self.max_linear_velocity)
        self.angular_velocity = clamp(
            command.angular.z,
            -self.max_angular_velocity, self.max_angular_velocity)
        self.last_command_time = self.get_clock().now()

    def initial_pose_callback(self, message):
        pose = message.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        if self.initialpose_uses_z:
            self.z = pose.position.z
        q = pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_command_time = None
        self.reference_segment = self.closest_reference_segment()
        self.executed_path.poses = []
        self.last_trace_position = None
        self.get_logger().info(
            'Offline base reset to x=%.2f y=%.2f z=%.2f yaw=%.1f deg' %
            (self.x, self.y, self.z, math.degrees(self.yaw)))

    def path_callback(self, message):
        if len(message.poses) < 2:
            return
        self.reference_path = [
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in message.poses]
        self.reference_segment = self.closest_reference_segment()
        self.get_logger().info(
            'Offline base received %d reference poses from %s' %
            (len(self.reference_path), self.path_topic))

    def closest_reference_segment(self):
        if len(self.reference_path) < 2:
            return 0
        best_index = min(
            range(len(self.reference_path)),
            key=lambda index: (
                (self.reference_path[index][0] - self.x) ** 2 +
                (self.reference_path[index][1] - self.y) ** 2 +
                (self.reference_path[index][2] - self.z) ** 2))
        return min(best_index, len(self.reference_path) - 2)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        if self.motion_mode == 'path':
            linear_velocity = self.follow_reference_path(
                self.path_following_speed * dt) / dt
            angular_velocity = self.path_end_angular_velocity(now)
        elif (
            self.last_command_time is None or
            (now - self.last_command_time).nanoseconds * 1e-9 >
            self.cmd_timeout
        ):
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            linear_velocity = self.linear_velocity
            angular_velocity = self.angular_velocity

        if self.motion_mode != 'path':
            self.yaw = normalize_angle(self.yaw + angular_velocity * dt)
            self.x += linear_velocity * math.cos(self.yaw) * dt
            self.y += linear_velocity * math.sin(self.yaw) * dt
            if self.follow_path_height:
                self.update_height_from_reference()
        elif angular_velocity != 0.0:
            self.yaw = normalize_angle(self.yaw + angular_velocity * dt)
        self.publish_state(now.to_msg(), linear_velocity, angular_velocity)

    def path_end_angular_velocity(self, now):
        path_finished = (
            len(self.reference_path) >= 2 and
            self.reference_segment >= len(self.reference_path) - 1)
        command_fresh = (
            self.last_command_time is not None and
            (now - self.last_command_time).nanoseconds * 1e-9 <=
            self.cmd_timeout)
        if self.align_at_path_end and path_finished and command_fresh:
            return self.angular_velocity
        return 0.0

    def update_height_from_reference(self):
        if len(self.reference_path) < 2:
            return
        begin = max(0, self.reference_segment - 2)
        end = min(len(self.reference_path) - 1, self.reference_segment + 50)
        best = None
        for index in range(begin, end):
            a = self.reference_path[index]
            b = self.reference_path[index + 1]
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            length_sq = dx * dx + dy * dy
            ratio = 0.0 if length_sq < 1e-8 else clamp(
                ((self.x - a[0]) * dx + (self.y - a[1]) * dy) / length_sq,
                0.0, 1.0)
            px = a[0] + ratio * dx
            py = a[1] + ratio * dy
            pz = a[2] + ratio * (b[2] - a[2])
            score = ((self.x - px) ** 2 + (self.y - py) ** 2 +
                     0.25 * (self.z - pz) ** 2)
            if best is None or score < best[0]:
                best = (score, index, pz)
        if best is not None:
            self.reference_segment = max(self.reference_segment, best[1])
            self.z = best[2]

    def follow_reference_path(self, distance_budget):
        travelled = 0.0
        while (
            distance_budget > 0.0 and
            self.reference_segment < len(self.reference_path) - 1
        ):
            target = self.reference_path[self.reference_segment + 1]
            dx = target[0] - self.x
            dy = target[1] - self.y
            dz = target[2] - self.z
            remaining = math.sqrt(dx * dx + dy * dy + dz * dz)
            if remaining < 1e-4:
                self.reference_segment += 1
                continue
            if math.hypot(dx, dy) > 1e-4:
                self.yaw = math.atan2(dy, dx)
            step = min(distance_budget, remaining)
            scale = step / remaining
            self.x += dx * scale
            self.y += dy * scale
            self.z += dz * scale
            distance_budget -= step
            travelled += step
            if step >= remaining - 1e-6:
                self.reference_segment += 1
        return travelled

    def publish_state(self, stamp, linear_velocity, angular_velocity):
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header = transform.header
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.position.z = self.z
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.twist.twist.linear.x = linear_velocity
        odometry.twist.twist.angular.z = angular_velocity
        self.odom_publisher.publish(odometry)
        self.publish_trace_and_error(stamp, qz, qw)

    def publish_trace_and_error(self, stamp, qz, qw):
        position = (self.x, self.y, self.z)
        if (
            self.last_trace_position is None or
            math.sqrt(sum(
                (a - b) ** 2 for a, b in
                zip(position, self.last_trace_position))) >= self.trace_spacing
        ):
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.position.z = self.z
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.executed_path.poses.append(pose)
            self.executed_path.header.stamp = stamp
            self.path_publisher.publish(self.executed_path)
            self.last_trace_position = position

        if self.reference_path:
            nearest = min(
                range(len(self.reference_path)),
                key=lambda index: (
                    (self.reference_path[index][0] - self.x) ** 2 +
                    (self.reference_path[index][1] - self.y) ** 2 +
                    (self.reference_path[index][2] - self.z) ** 2))
            point = self.reference_path[nearest]
            error = math.sqrt((point[0] - self.x) ** 2 +
                              (point[1] - self.y) ** 2 +
                              (point[2] - self.z) ** 2)
            message = Float32MultiArray()
            message.data = [
                error,
                float(nearest),
                float(nearest) / max(1.0, len(self.reference_path) - 1.0),
            ]
            self.error_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = OfflineBaseSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
