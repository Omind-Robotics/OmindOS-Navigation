#!/usr/bin/env python3

import math
import os

import rospy
from nav_msgs.msg import Odometry


class MappingTrajectoryRecorder:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry_loc")
        self.output_file = rospy.get_param("~output_file", "")
        self.min_interval = float(rospy.get_param("~min_interval", 0.1))
        self.min_distance = float(rospy.get_param("~min_distance", 0.02))
        self.append = bool(rospy.get_param("~append", False))

        if not self.output_file:
            raise rospy.ROSInitException("~output_file must be set")

        output_dir = os.path.dirname(os.path.abspath(self.output_file))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        mode = "a" if self.append else "w"
        self.file_handle = open(self.output_file, mode, buffering=1)
        self.last_stamp = None
        self.last_position = None
        self.count = 0

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=100)
        rospy.on_shutdown(self.close)

        rospy.loginfo(
            "Recording mapping trajectory from %s to %s",
            self.odom_topic,
            self.output_file,
        )

    def odom_callback(self, msg):
        stamp = msg.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()
        stamp_sec = stamp.to_sec()

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        if self.last_stamp is not None:
            if stamp_sec - self.last_stamp < self.min_interval:
                return
            if self.distance(position, self.last_position) < self.min_distance:
                return

        self.file_handle.write(
            "{:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f}\n".format(
                stamp_sec,
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        )

        self.last_stamp = stamp_sec
        self.last_position = (position.x, position.y, position.z)
        self.count += 1

        if self.count % 100 == 0:
            rospy.loginfo("Recorded %d mapping trajectory poses", self.count)

    @staticmethod
    def distance(position, last_position):
        if last_position is None:
            return float("inf")
        dx = position.x - last_position[0]
        dy = position.y - last_position[1]
        dz = position.z - last_position[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def close(self):
        if getattr(self, "file_handle", None) and not self.file_handle.closed:
            self.file_handle.flush()
            self.file_handle.close()
        rospy.loginfo("Saved %d mapping trajectory poses to %s", self.count, self.output_file)


if __name__ == "__main__":
    rospy.init_node("mapping_trajectory_recorder")
    recorder = MappingTrajectoryRecorder()
    rospy.spin()
