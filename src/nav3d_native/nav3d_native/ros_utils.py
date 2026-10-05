import math
from geometry_msgs.msg import Quaternion
from .algorithms import Pose2D


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose2d_from_pose(pose) -> Pose2D:
    return Pose2D(pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
