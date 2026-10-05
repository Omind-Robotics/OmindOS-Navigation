from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav3d_hardware_bringup',
            executable='go2_odom_bridge',
            name='go2_odom_bridge',
            output='screen',
            parameters=[{
                'unitree_interface': 'enp86s0',
                'unitree_peer': '192.168.123.161',
                'unitree_domain_id': 0,
                'unitree_topic': 'rt/lf/sportmodestate',
                'allow_multicast': 'spdp',
                'output_topic': '/go2/odom_raw',
                'frame_id': 'go2_odom_raw',
                'child_frame_id': 'go2_base_raw',
            }],
        ),
    ])
