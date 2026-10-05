#!/usr/bin/env python3
"""ROS 2 port of the original Go2 /cmd_vel SportClient bridge."""

import threading
import time
from xml.sax.saxutils import escape

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


def _clamp(value, limit):
    return max(-limit, min(value, limit))


def _shape_nonzero_command(value, minimum, maximum, zero_epsilon=1e-4):
    """Keep zero as zero and map an active command into the usable band."""
    if abs(value) <= zero_epsilon:
        return 0.0
    magnitude = max(minimum, min(abs(value), maximum))
    return magnitude if value > 0.0 else -magnitude


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


class Go2BaseControllerRos2(Node):
    def __init__(self):
        super().__init__('go2_base_controller_ros2')
        self.declare_parameter('unitree_interface', 'enp86s0')
        self.declare_parameter('unitree_peer', '192.168.123.161')
        self.declare_parameter('unitree_domain_id', 0)
        self.declare_parameter('allow_multicast', 'spdp')
        self.declare_parameter('command_timeout_seconds', 1.0)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('sport_client_timeout_seconds', 10.0)
        self.declare_parameter('max_linear_velocity', 0.9)
        self.declare_parameter('max_lateral_velocity', 0.9)
        self.declare_parameter('max_angular_velocity', 1.4)
        self.declare_parameter('min_effective_linear_velocity', 0.0)
        self.declare_parameter('min_effective_angular_velocity', 0.0)
        self.declare_parameter('zero_command_epsilon', 1e-4)

        interface = str(self.get_parameter('unitree_interface').value)
        peer = str(self.get_parameter('unitree_peer').value)
        domain_id = int(self.get_parameter('unitree_domain_id').value)
        allow_multicast = str(self.get_parameter('allow_multicast').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout_seconds').value)
        control_frequency = float(
            self.get_parameter('control_frequency').value)
        client_timeout = float(
            self.get_parameter('sport_client_timeout_seconds').value)
        self.max_linear_velocity = float(
            self.get_parameter('max_linear_velocity').value)
        self.max_lateral_velocity = float(
            self.get_parameter('max_lateral_velocity').value)
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value)
        self.min_effective_linear_velocity = float(
            self.get_parameter('min_effective_linear_velocity').value)
        self.min_effective_angular_velocity = float(
            self.get_parameter('min_effective_angular_velocity').value)
        self.zero_command_epsilon = float(
            self.get_parameter('zero_command_epsilon').value)
        if not interface:
            raise ValueError('unitree_interface must not be empty')
        if self.command_timeout <= 0.0 or control_frequency <= 0.0:
            raise ValueError('timeouts and frequencies must be positive')
        if (self.max_linear_velocity <= 0.0 or
                self.max_lateral_velocity < 0.0 or
                self.max_angular_velocity <= 0.0):
            raise ValueError('velocity limits must be positive or zero')
        if (self.min_effective_linear_velocity < 0.0 or
                self.min_effective_angular_velocity < 0.0 or
                self.zero_command_epsilon < 0.0):
            raise ValueError(
                'minimum velocities and epsilon must be nonnegative')
        if (self.min_effective_linear_velocity >
                self.max_linear_velocity or
                self.min_effective_angular_velocity >
                self.max_angular_velocity):
            raise ValueError('minimum velocity must not exceed maximum')

        import unitree_sdk2py.core.channel as unitree_channel
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        unitree_channel.ChannelConfigHasInterface = _cyclonedds_config(
            interface, peer, allow_multicast)
        unitree_channel.ChannelFactoryInitialize(domain_id, interface)
        self.client = SportClient()
        self.client.SetTimeout(client_timeout)
        self.client.Init()

        self.lock = threading.Lock()
        self.last_command = Twist()
        self.last_command_time = time.monotonic()
        self.emergency_stop = False
        self.closed = False
        self.loop_count = 0
        self.create_subscription(Twist, '/cmd_vel', self._command, 10)
        self.create_subscription(
            Bool, '/emergency_stop', self._emergency_stop, 10)
        self.timer = self.create_timer(
            1.0 / control_frequency, self._control_loop)
        self.get_logger().info(
            f'Go2 SportClient bridge ready: iface={interface}, '
            f'peer={peer or "none"}, domain={domain_id}')

    def _command(self, message):
        with self.lock:
            self.last_command = message
            self.last_command_time = time.monotonic()

    def _emergency_stop(self, message):
        if not message.data:
            return
        with self.lock:
            self.emergency_stop = True
        self.get_logger().warning('EMERGENCY STOP ACTIVATED')
        try:
            code = int(self.client.StopMove())
            self.get_logger().info(f'StopMove return_code={code}')
        except Exception as error:  # SDK exception boundary
            self.get_logger().error(f'StopMove failed: {error}')

    def _control_loop(self):
        if self.closed:
            return
        self.loop_count += 1
        with self.lock:
            age = time.monotonic() - self.last_command_time
            if self.emergency_stop or age > self.command_timeout:
                vx = vy = yaw = 0.0
            else:
                vx = _shape_nonzero_command(
                    float(self.last_command.linear.x),
                    self.min_effective_linear_velocity,
                    self.max_linear_velocity,
                    self.zero_command_epsilon)
                vy = _clamp(
                    float(self.last_command.linear.y),
                    self.max_lateral_velocity)
                yaw = _shape_nonzero_command(
                    float(self.last_command.angular.z),
                    self.min_effective_angular_velocity,
                    self.max_angular_velocity,
                    self.zero_command_epsilon)
        try:
            started = time.monotonic()
            code = int(self.client.Move(vx, vy, yaw))
            duration = time.monotonic() - started
            if code != 0:
                self.get_logger().error(
                    f'Move({vx:.3f}, {vy:.3f}, {yaw:.3f}) '
                    f'return_code={code}')
            if duration > 0.1:
                self.get_logger().warning(
                    f'SLOW Move(): {duration * 1000.0:.1f} ms')
            if self.loop_count % 50 == 0:
                self.get_logger().info(
                    f'Sending cmd_vel: vx={vx:.2f}, vy={vy:.2f}, '
                    f'yaw={yaw:.2f}, age={age:.3f}s')
        except Exception as error:  # SDK exception boundary
            self.get_logger().error(f'Move failed: {error}')

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            for _ in range(10):
                self.client.Move(0.0, 0.0, 0.0)
                time.sleep(0.02)
            code = int(self.client.StopMove())
            self.get_logger().info(
                f'All movements stopped; StopMove return_code={code}')
        except Exception as error:  # SDK exception boundary
            self.get_logger().error(f'Failed to stop movements: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Go2BaseControllerRos2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
