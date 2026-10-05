#!/usr/bin/env python3
"""ROS 2 H4 Go2 command gate with a permanently locked motion boundary."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from go2_command_gate_core import GateConfig, evaluate_gate, validate_config


# Deliberately not a ROS parameter. This process does not import Unitree SDK.
MOTION_STAGE_UNLOCKED = False


class Go2CommandGateDryRun(Node):
    def __init__(self):
        super().__init__('go2_command_gate_dry_run')
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('enable_topic', '/nav/control_enabled')
        self.declare_parameter('emergency_stop_topic', '/nav/emergency_stop')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_timeout_seconds', 0.15)
        self.declare_parameter('safety_timeout_seconds', 0.15)
        self.declare_parameter('max_linear_velocity_m_s', 0.05)
        self.declare_parameter('max_angular_velocity_rad_s', 0.30)
        self.declare_parameter('zero_epsilon', 1.0e-6)

        self.config = GateConfig(
            command_timeout_seconds=float(self.get_parameter(
                'command_timeout_seconds').value),
            safety_timeout_seconds=float(self.get_parameter(
                'safety_timeout_seconds').value),
            max_linear_velocity_m_s=float(self.get_parameter(
                'max_linear_velocity_m_s').value),
            max_angular_velocity_rad_s=float(self.get_parameter(
                'max_angular_velocity_rad_s').value),
            zero_epsilon=float(self.get_parameter('zero_epsilon').value),
        )
        validate_config(self.config)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate_hz < 20.0 or publish_rate_hz > 100.0:
            raise ValueError('publish_rate_hz must be in [20, 100]')

        self.command = (0.0,) * 6
        self.command_time = None
        self.enabled = False
        self.enable_time = None
        self.emergency_stop = True
        self.estop_time = None
        self.last_status = None

        command_topic = str(self.get_parameter('command_topic').value)
        enable_topic = str(self.get_parameter('enable_topic').value)
        estop_topic = str(self.get_parameter('emergency_stop_topic').value)
        self.create_subscription(Twist, command_topic, self._command, 1)
        self.create_subscription(Bool, enable_topic, self._enable, 1)
        self.create_subscription(Bool, estop_topic, self._estop, 1)
        self.status_pub = self.create_publisher(
            String, '/nav/h4_gate/status', 1)
        self.input_valid_pub = self.create_publisher(
            Bool, '/nav/h4_gate/input_valid', 1)
        self.output_pub = self.create_publisher(
            Twist, '/nav/h4_gate/zero_output', 1)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._tick)
        self.get_logger().warning(
            'H4 dry-run gate started with motion stage locked; no Unitree SDK '
            'is imported and zero_output is always all-zero')

    @staticmethod
    def _now():
        return time.monotonic()

    def _command(self, message):
        self.command = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        self.command_time = self._now()

    def _enable(self, message):
        self.enabled = bool(message.data)
        self.enable_time = self._now()

    def _estop(self, message):
        self.emergency_stop = bool(message.data)
        self.estop_time = self._now()

    @staticmethod
    def _age(now, stamp):
        return None if stamp is None else now - stamp

    def _tick(self):
        now = self._now()
        decision = evaluate_gate(
            command=self.command,
            command_age=self._age(now, self.command_time),
            enabled=self.enabled,
            enable_age=self._age(now, self.enable_time),
            emergency_stop=self.emergency_stop,
            estop_age=self._age(now, self.estop_time),
            config=self.config,
            motion_stage_unlocked=MOTION_STAGE_UNLOCKED,
        )
        self.status_pub.publish(String(data=decision.status))
        self.input_valid_pub.publish(Bool(data=decision.input_valid))
        self.output_pub.publish(Twist())
        if decision.status != self.last_status:
            self.get_logger().info(f'H4 gate status: {decision.status}')
            self.last_status = decision.status

    def close(self):
        if not rclpy.ok():
            return
        for _ in range(3):
            self.output_pub.publish(Twist())
            self.input_valid_pub.publish(Bool(data=False))
        self.status_pub.publish(String(data='SHUTDOWN_FAIL_CLOSED'))


def main(args=None):
    rclpy.init(args=args)
    node = Go2CommandGateDryRun()
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
