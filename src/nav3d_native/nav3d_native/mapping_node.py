from __future__ import annotations
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from .algorithms import LogOddsGrid
from .ros_utils import pose2d_from_pose


MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)


class MappingNode(Node):
    def __init__(self):
        super().__init__('nav3d_mapper')
        self.declare_parameter('pose_topic', '/ground_truth/odom')
        self.declare_parameter('map_output', '/tmp/nav3d_demo_map')
        self.grid = LogOddsGrid(120, 100, 0.1, -6.0, -5.0)
        self.latest_pose = None
        self.scan_count = 0
        self.map_output = str(self.get_parameter('map_output').value)
        self.publisher = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self.create_subscription(
            Odometry,
            str(self.get_parameter('pose_topic').value),
            self._pose_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan', self._scan_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.5, self._publish)

    def _pose_callback(self, msg: Odometry):
        self.latest_pose = pose2d_from_pose(msg.pose.pose)

    def _scan_callback(self, msg: LaserScan):
        if self.latest_pose is None:
            return
        self.grid.update_scan(
            self.latest_pose,
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max)
        self.scan_count += 1

    def _message(self) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.occupancy()
        return msg

    def _save(self, data):
        if not self.map_output or self.scan_count < 10:
            return
        base = Path(self.map_output)
        base.parent.mkdir(parents=True, exist_ok=True)
        pgm = base.with_suffix('.pgm')
        yaml = base.with_suffix('.yaml')
        tmp = pgm.with_suffix('.pgm.tmp')
        with tmp.open('wb') as stream:
            header = f'P5\n{self.grid.width} {self.grid.height}\n255\n'
            stream.write(header.encode())
            for y in range(self.grid.height - 1, -1, -1):
                row = bytearray()
                for x in range(self.grid.width):
                    value = data[y * self.grid.width + x]
                    pixel = 205 if value < 0 else (
                        0 if value >= 65 else 254)
                    row.append(pixel)
                stream.write(row)
        os.replace(tmp, pgm)
        yaml.write_text(
            f'image: {pgm.name}\nresolution: {self.grid.resolution}\n'
            f'origin: [{self.grid.origin_x}, {self.grid.origin_y}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n',
            encoding='utf-8')

    def _publish(self):
        msg = self._message()
        self.publisher.publish(msg)
        self._save(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
