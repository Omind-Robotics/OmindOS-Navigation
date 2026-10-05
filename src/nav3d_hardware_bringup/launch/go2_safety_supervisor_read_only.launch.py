from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav3d_hardware_bringup',
            executable='go2_safety_supervisor',
            name='go2_safety_supervisor',
            output='screen',
            parameters=[{
                'unitree_interface': 'enp86s0',
                'unitree_peer': '192.168.123.161',
                'unitree_domain_id': 0,
                'lowstate_topic': 'rt/lf/lowstate',
                'allow_multicast': 'spdp',
                'publish_rate_hz': 20.0,
                'lowstate_timeout_seconds': 0.25,
                'operator_timeout_seconds': 0.30,
                'axis_takeover_threshold': 0.15,
                'takeover_reset_hold_seconds': 1.0,
                'remote_online_prefix': [85, 81],
            }],
        ),
    ])
