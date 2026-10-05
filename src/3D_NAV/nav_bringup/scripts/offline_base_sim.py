#!/usr/bin/env python3
"""Kinematic base simulator for exercising the navigation stack without hardware."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OfflineBaseSim:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.update_rate = rospy.get_param("~update_rate", 50.0)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.35)
        self.max_linear_velocity = rospy.get_param("~max_linear_velocity", 0.9)
        self.max_angular_velocity = rospy.get_param("~max_angular_velocity", 1.4)
        self.motion_mode = rospy.get_param("~motion_mode", "path")
        self.path_topic = rospy.get_param("~path_topic", "/path_smooth")
        self.path_following_speed = rospy.get_param("~path_following_speed", 0.35)
        self.initialpose_uses_z = rospy.get_param("~initialpose_uses_z", False)

        self.x = rospy.get_param("~initial_x", 0.0)
        self.y = rospy.get_param("~initial_y", 0.0)
        self.z = rospy.get_param("~initial_z", 0.0)
        self.yaw = rospy.get_param("~initial_yaw", 0.0)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_command_time = rospy.Time(0)
        self.last_update_time = rospy.Time.now()
        self.reference_path = []
        self.reference_segment = 0

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.command_callback, queue_size=10)
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                         self.initial_pose_callback, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.update)
        rospy.loginfo(
            "Offline base simulator ready in %s mode. Set /initialpose in RViz, then send a "
            "3D Nav Goal. It never contacts robot hardware.", self.motion_mode)

    def command_callback(self, command):
        self.linear_velocity = clamp(command.linear.x,
                                     -self.max_linear_velocity,
                                     self.max_linear_velocity)
        self.angular_velocity = clamp(command.angular.z,
                                      -self.max_angular_velocity,
                                      self.max_angular_velocity)
        self.last_command_time = rospy.Time.now()

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
        self.last_command_time = rospy.Time(0)
        self.reference_segment = self.closest_reference_segment()
        rospy.loginfo("Offline base reset to x=%.2f y=%.2f z=%.2f yaw=%.1f deg",
                      self.x, self.y, self.z, math.degrees(self.yaw))

    def path_callback(self, message):
        if len(message.poses) < 2:
            return

        self.reference_path = [(pose.pose.position.x,
                                pose.pose.position.y,
                                pose.pose.position.z)
                               for pose in message.poses]
        self.reference_segment = self.closest_reference_segment()
        rospy.loginfo("Offline base received %d reference poses from %s",
                      len(self.reference_path), self.path_topic)

    def closest_reference_segment(self):
        if len(self.reference_path) < 2:
            return 0

        best_index = 0
        best_distance_sq = float("inf")
        for index, point in enumerate(self.reference_path):
            distance_sq = ((point[0] - self.x) ** 2 +
                           (point[1] - self.y) ** 2 +
                           (point[2] - self.z) ** 2)
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_index = index
        return min(best_index, len(self.reference_path) - 2)

    def update(self, _event):
        now = rospy.Time.now()
        dt = (now - self.last_update_time).to_sec()
        self.last_update_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        if self.motion_mode == "path":
            linear_velocity = self.follow_reference_path(self.path_following_speed * dt) / dt
            angular_velocity = 0.0
        elif (now - self.last_command_time).to_sec() > self.cmd_timeout:
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            linear_velocity = self.linear_velocity
            angular_velocity = self.angular_velocity

        if self.motion_mode != "path":
            self.yaw = normalize_angle(self.yaw + angular_velocity * dt)
            self.x += linear_velocity * math.cos(self.yaw) * dt
            self.y += linear_velocity * math.sin(self.yaw) * dt
        self.publish_state(now, linear_velocity, angular_velocity)

    def follow_reference_path(self, distance_budget):
        travelled = 0.0
        while distance_budget > 0.0 and self.reference_segment < len(self.reference_path) - 1:
            target = self.reference_path[self.reference_segment + 1]
            dx = target[0] - self.x
            dy = target[1] - self.y
            dz = target[2] - self.z
            remaining = math.sqrt(dx * dx + dy * dy + dz * dz)

            if remaining < 1e-4:
                self.reference_segment += 1
                continue

            horizontal_distance = math.hypot(dx, dy)
            if horizontal_distance > 1e-4:
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
        half_yaw = self.yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

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



if __name__ == "__main__":
    rospy.init_node("offline_base_sim")
    OfflineBaseSim()
    rospy.spin()
