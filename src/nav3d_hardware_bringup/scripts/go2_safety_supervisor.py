#!/usr/bin/env python3
"""Read-only ROS 2 Go2 remote-priority safety supervisor."""

import threading
import time
from xml.sax.saxutils import escape

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from go2_safety_supervisor_core import SafetyConfig
from go2_safety_supervisor_core import SafetySupervisorCore


# Deliberately not a ROS parameter. No motion client exists in this process.
MOTION_STAGE_UNLOCKED = False


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


class Go2SafetySupervisor(Node):
    def __init__(self):
        super().__init__('go2_safety_supervisor')
        self.declare_parameter('unitree_interface', 'enp86s0')
        self.declare_parameter('unitree_peer', '192.168.123.161')
        self.declare_parameter('unitree_domain_id', 0)
        self.declare_parameter('lowstate_topic', 'rt/lf/lowstate')
        self.declare_parameter('allow_multicast', 'spdp')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('lowstate_timeout_seconds', 0.25)
        self.declare_parameter('operator_timeout_seconds', 0.30)
        self.declare_parameter('axis_takeover_threshold', 0.15)
        self.declare_parameter('takeover_reset_hold_seconds', 1.0)
        self.declare_parameter('remote_online_prefix', [85, 81])

        interface = str(self.get_parameter('unitree_interface').value)
        peer = str(self.get_parameter('unitree_peer').value)
        domain_id = int(self.get_parameter('unitree_domain_id').value)
        lowstate_topic = str(self.get_parameter('lowstate_topic').value)
        allow_multicast = str(self.get_parameter('allow_multicast').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        prefix = tuple(int(value) for value in self.get_parameter(
            'remote_online_prefix').value)
        if not interface:
            raise ValueError('unitree_interface must not be empty')
        if publish_rate_hz < 20.0 or publish_rate_hz > 100.0:
            raise ValueError('publish_rate_hz must be in [20, 100]')

        config = SafetyConfig(
            lowstate_timeout_seconds=float(self.get_parameter(
                'lowstate_timeout_seconds').value),
            operator_timeout_seconds=float(self.get_parameter(
                'operator_timeout_seconds').value),
            axis_takeover_threshold=float(self.get_parameter(
                'axis_takeover_threshold').value),
            takeover_reset_hold_seconds=float(self.get_parameter(
                'takeover_reset_hold_seconds').value),
            remote_online_prefix=prefix,
        )
        self.core = SafetySupervisorCore(config)
        self.lock = threading.Lock()
        self._closing = False
        self._last_status = None

        self.lowstate_ok_pub = self.create_publisher(
            Bool, '/nav/safety/lowstate_ok', 1)
        self.remote_online_pub = self.create_publisher(
            Bool, '/nav/safety/remote_online', 1)
        self.remote_takeover_pub = self.create_publisher(
            Bool, '/nav/safety/remote_takeover', 1)
        self.remote_soft_estop_pub = self.create_publisher(
            Bool, '/nav/safety/remote_soft_estop_input', 1)
        self.remote_motion_permit_pub = self.create_publisher(
            Bool, '/nav/safety/remote_motion_permit', 1)
        self.estop_pub = self.create_publisher(
            Bool, '/nav/emergency_stop', 1)
        self.enable_pub = self.create_publisher(
            Bool, '/nav/control_enabled', 1)
        self.status_pub = self.create_publisher(
            String, '/nav/safety/status', 1)
        self.create_subscription(
            Bool, '/nav/operator_auto_request', self._operator_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._tick)

        import unitree_sdk2py.core.channel as unitree_channel
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

        unitree_channel.ChannelConfigHasInterface = _cyclonedds_config(
            interface, peer, allow_multicast)
        unitree_channel.ChannelFactoryInitialize(domain_id, interface)
        self.subscriber = ChannelSubscriber(lowstate_topic, LowState_)
        self.subscriber.Init(self._lowstate_callback, 10)
        self.get_logger().warning(
            'read-only Go2 safety supervisor started; '
            'MOTION_STAGE_UNLOCKED=False; no SportClient exists')

    @staticmethod
    def _now():
        return time.monotonic()

    def _operator_callback(self, message):
        with self.lock:
            self.core.observe_operator(bool(message.data), self._now())

    def _lowstate_callback(self, message):
        if self._closing:
            return
        now = self._now()
        try:
            with self.lock:
                _, events = self.core.observe_lowstate(
                    message.wireless_remote, now)
        except (TypeError, ValueError) as error:
            with self.lock:
                self.core.observe_invalid_lowstate(now)
            self.get_logger().error(
                f'invalid wireless_remote rejected: {error}')
            return
        for event in events:
            self.get_logger().warning(event)

    def _tick(self):
        with self.lock:
            decision = self.core.evaluate(
                self._now(),
                motion_stage_unlocked=MOTION_STAGE_UNLOCKED)
        self.lowstate_ok_pub.publish(Bool(data=decision.lowstate_ok))
        self.remote_online_pub.publish(Bool(data=decision.remote_online))
        self.remote_takeover_pub.publish(
            Bool(data=decision.remote_takeover))
        self.remote_soft_estop_pub.publish(
            Bool(data=decision.remote_soft_estop_input))
        self.remote_motion_permit_pub.publish(
            Bool(data=decision.remote_motion_permit))
        self.estop_pub.publish(Bool(data=decision.emergency_stop))
        self.enable_pub.publish(Bool(data=decision.control_enabled))
        self.status_pub.publish(String(data=decision.status))
        if decision.status != self._last_status:
            self.get_logger().info(
                f'safety status: {decision.status}; '
                f'control_enabled={decision.control_enabled}')
            self._last_status = decision.status

    def close(self):
        if self._closing:
            return
        self._closing = True
        if self.subscriber is not None:
            self.subscriber.Close()
            self.subscriber = None
        if not rclpy.ok():
            return
        for _ in range(3):
            self.enable_pub.publish(Bool(data=False))
            self.estop_pub.publish(Bool(data=True))
            self.remote_motion_permit_pub.publish(Bool(data=False))
        self.status_pub.publish(String(data='SHUTDOWN_FAIL_CLOSED'))


def main(args=None):
    rclpy.init(args=args)
    node = Go2SafetySupervisor()
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
