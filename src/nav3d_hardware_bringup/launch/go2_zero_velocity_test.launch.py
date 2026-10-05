from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    confirm_token = LaunchConfiguration('confirm_token')
    return LaunchDescription([
        DeclareLaunchArgument('confirm_token', default_value='BLOCKED'),
        Node(
            package='nav3d_hardware_bringup',
            executable='go2_zero_velocity_test',
            name='go2_zero_velocity_test',
            output='screen',
            arguments=[
                '--interface', 'enp86s0',
                '--peer', '192.168.123.161',
                '--domain-id', '0',
                '--allow-multicast', 'spdp',
                '--confirm-token', confirm_token,
            ],
        ),
    ])
