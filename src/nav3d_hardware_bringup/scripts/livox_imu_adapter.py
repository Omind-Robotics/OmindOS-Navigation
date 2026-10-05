#!/usr/bin/env python3
"""Convert the MID360 vendor IMU stream to a standards-compliant ROS IMU."""

from sensor_msgs.msg import Imu

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


STANDARD_GRAVITY = 9.80665


def convert_imu(raw: Imu) -> Imu:
    """Convert acceleration to SI and mark the absent orientation."""
    message = Imu()
    message.header = raw.header

    # MID360 supplies angular velocity and acceleration, but no orientation.
    # Keep a benign identity value and use the ROS covariance sentinel so
    # consumers know that the orientation fields must be ignored.
    message.orientation.w = 1.0
    message.orientation_covariance[0] = -1.0

    message.angular_velocity = raw.angular_velocity
    message.angular_velocity_covariance = raw.angular_velocity_covariance

    message.linear_acceleration.x = (
        raw.linear_acceleration.x * STANDARD_GRAVITY)
    message.linear_acceleration.y = (
        raw.linear_acceleration.y * STANDARD_GRAVITY)
    message.linear_acceleration.z = (
        raw.linear_acceleration.z * STANDARD_GRAVITY)
    message.linear_acceleration_covariance = (
        raw.linear_acceleration_covariance)
    return message


class LivoxImuAdapter(Node):
    def __init__(self):
        super().__init__('livox_imu_adapter')
        self.declare_parameter('input_topic', '/livox/imu_raw')
        self.declare_parameter('output_topic', '/livox/imu')
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        # Offer reliable delivery for estimators such as FAST-LIO. A reliable
        # publisher also remains compatible with best-effort sensor consumers.
        self.publisher = self.create_publisher(Imu, output_topic, 20)
        self.subscription = self.create_subscription(
            Imu, input_topic, self._callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Livox IMU SI adapter: {input_topic} (g) -> '
            f'{output_topic} (m/s^2); orientation unavailable')

    def _callback(self, raw: Imu):
        self.publisher.publish(convert_imu(raw))


def main(args=None):
    rclpy.init(args=args)
    node = LivoxImuAdapter()
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
