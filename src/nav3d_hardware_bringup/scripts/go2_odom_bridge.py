#!/usr/bin/env python3
"""Read-only Unitree SportModeState to ROS 2 Odometry bridge."""

import math
from xml.sax.saxutils import escape

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import unitree_sdk2py.core.channel as unitree_channel
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


def _cyclonedds_config(interface, peer, allow_multicast):
    peer_section = ''
    if peer:
        peer_section = (
            '<Discovery><Peers>'
            f'<Peer Address="{escape(peer)}"/>'
            '</Peers></Discovery>'
        )
    return f'''<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="{escape(interface)}"
          priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>{escape(allow_multicast)}</AllowMulticast>
    </General>
    {peer_section}
  </Domain>
</CycloneDDS>'''


def _set_diagonal(covariance, values):
    for index, value in zip((0, 7, 14, 21, 28, 35), values):
        covariance[index] = value


class Go2OdomBridge(Node):
    def __init__(self):
        super().__init__('go2_odom_bridge')
        self.declare_parameter('unitree_interface', 'enp86s0')
        self.declare_parameter('unitree_peer', '192.168.123.161')
        self.declare_parameter('unitree_domain_id', 0)
        self.declare_parameter('unitree_topic', 'rt/lf/sportmodestate')
        self.declare_parameter('allow_multicast', 'spdp')
        self.declare_parameter('output_topic', '/go2/odom_raw')
        self.declare_parameter('frame_id', 'go2_odom_raw')
        self.declare_parameter('child_frame_id', 'go2_base_raw')

        interface = str(self.get_parameter('unitree_interface').value)
        peer = str(self.get_parameter('unitree_peer').value)
        domain_id = int(self.get_parameter('unitree_domain_id').value)
        self.unitree_topic = str(self.get_parameter('unitree_topic').value)
        allow_multicast = str(self.get_parameter('allow_multicast').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)

        if not interface:
            raise ValueError('unitree_interface must not be empty')

        config = _cyclonedds_config(interface, peer, allow_multicast)
        unitree_channel.ChannelConfigHasInterface = config
        unitree_channel.ChannelFactoryInitialize(domain_id, interface)

        self.publisher = self.create_publisher(
            Odometry, output_topic, qos_profile_sensor_data)
        self.subscriber = ChannelSubscriber(
            self.unitree_topic, SportModeState_)
        self._closing = False
        self._message_count = 0
        self._last_log_ns = self.get_clock().now().nanoseconds
        self.subscriber.Init(self._state_callback, 10)

        self.get_logger().info(
            f'read-only bridge started: DDS {self.unitree_topic} on '
            f'{interface} peer={peer or "none"} domain={domain_id} -> '
            f'ROS 2 {output_topic}; TF disabled')

    def _state_callback(self, state):
        if self._closing:
            return
        values = (
            list(state.position) + list(state.velocity) +
            list(state.imu_state.quaternion) + [state.yaw_speed]
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warning(
                'discarded non-finite SportModeState sample')
            return

        quaternion = [float(value) for value in state.imu_state.quaternion]
        quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
        if quaternion_norm < 1.0e-6:
            self.get_logger().warning('discarded zero-norm IMU quaternion')
            return
        quaternion = [value / quaternion_norm for value in quaternion]

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.child_frame_id = self.child_frame_id
        message.pose.pose.position.x = float(state.position[0])
        message.pose.pose.position.y = float(state.position[1])
        message.pose.pose.position.z = float(state.position[2])
        # Unitree SportModeState quaternion order is [w, x, y, z].
        message.pose.pose.orientation.w = quaternion[0]
        message.pose.pose.orientation.x = quaternion[1]
        message.pose.pose.orientation.y = quaternion[2]
        message.pose.pose.orientation.z = quaternion[3]
        message.twist.twist.linear.x = float(state.velocity[0])
        message.twist.twist.linear.y = float(state.velocity[1])
        message.twist.twist.linear.z = float(state.velocity[2])
        message.twist.twist.angular.x = float(state.imu_state.gyroscope[0])
        message.twist.twist.angular.y = float(state.imu_state.gyroscope[1])
        message.twist.twist.angular.z = float(state.yaw_speed)
        _set_diagonal(
            message.pose.covariance,
            (0.25, 0.25, 1.0, 0.1, 0.1, 0.25),
        )
        _set_diagonal(
            message.twist.covariance,
            (0.25, 0.25, 1.0, 0.1, 0.1, 0.25),
        )
        self.publisher.publish(message)

        self._message_count += 1
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns >= 5_000_000_000:
            self.get_logger().info(
                f'published {self._message_count} raw odometry samples')
            self._last_log_ns = now_ns

    def close(self):
        if self._closing:
            return
        self._closing = True
        if self.subscriber is not None:
            self.subscriber.Close()
            self.subscriber = None
        if rclpy.ok():
            self.get_logger().info(
                f'read-only bridge stopped after '
                f'{self._message_count} samples')


def main(args=None):
    rclpy.init(args=args)
    node = Go2OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
