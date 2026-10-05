#!/usr/bin/env python3
"""Compose FAST-LIO's IMU pose with the accepted base-to-IMU extrinsic."""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import yaml


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1.0e-12:
        raise ValueError('zero quaternion')
    return tuple(value / norm for value in q)


def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q, vector):
    result = quat_multiply(quat_multiply(q, (*vector, 0.0)),
                           quat_conjugate(q))
    return result[:3]


def compose_base_pose(position_wi, orientation_wi, position_bi,
                      orientation_bi):
    """Return T_world_base from T_world_imu and T_base_imu."""
    q_wi = quat_normalize(orientation_wi)
    q_bi = quat_normalize(orientation_bi)
    q_ib = quat_conjugate(q_bi)
    position_ib = rotate(q_ib, tuple(-value for value in position_bi))
    offset_world = rotate(q_wi, position_ib)
    position_wb = tuple(position_wi[i] + offset_world[i] for i in range(3))
    orientation_wb = quat_normalize(quat_multiply(q_wi, q_ib))
    return position_wb, orientation_wb


def _load_base_to_imu(path):
    with open(path, encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if document.get('runtime_allowed') is not True:
        raise RuntimeError('extrinsics are not runtime-approved')
    transforms = document['transforms']
    chain = [
        transforms['base_link_to_body'],
        transforms['body_to_lidar'],
        transforms['lidar_to_imu'],
    ]
    position = (0.0, 0.0, 0.0)
    orientation = (0.0, 0.0, 0.0, 1.0)
    for transform in chain:
        translation = tuple(float(v) for v in transform['translation_m'])
        rotation = quat_normalize(tuple(
            float(v) for v in transform['rotation_xyzw']))
        translated = rotate(orientation, translation)
        position = tuple(position[i] + translated[i] for i in range(3))
        orientation = quat_normalize(quat_multiply(orientation, rotation))
    return position, orientation


class LioBaseOdomAdapter(Node):
    def __init__(self):
        super().__init__('lio_base_odom_adapter')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('input_topic', '/lio/odometry_raw')
        self.declare_parameter('output_topic', '/lio/odom')
        calibration_file = str(self.get_parameter('calibration_file').value)
        if not calibration_file:
            raise RuntimeError('calibration_file is required')
        self.position_bi, self.orientation_bi = _load_base_to_imu(
            calibration_file)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(Odometry, output_topic, 20)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self._callback, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            f'LIO pose adapter: {input_topic} (lio_imu) -> '
            f'{output_topic} (base_link)')

    def _callback(self, raw):
        raw_position = raw.pose.pose.position
        raw_orientation = raw.pose.pose.orientation
        position, orientation = compose_base_pose(
            (raw_position.x, raw_position.y, raw_position.z),
            (raw_orientation.x, raw_orientation.y,
             raw_orientation.z, raw_orientation.w),
            self.position_bi, self.orientation_bi)
        output = Odometry()
        output.header = raw.header
        output.header.frame_id = 'lio_odom'
        output.child_frame_id = 'base_link'
        output.pose = raw.pose
        output.pose.pose.position.x = position[0]
        output.pose.pose.position.y = position[1]
        output.pose.pose.position.z = position[2]
        output.pose.pose.orientation.x = orientation[0]
        output.pose.pose.orientation.y = orientation[1]
        output.pose.pose.orientation.z = orientation[2]
        output.pose.pose.orientation.w = orientation[3]
        output.twist = raw.twist
        self.publisher.publish(output)

        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = output.child_frame_id
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = LioBaseOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
