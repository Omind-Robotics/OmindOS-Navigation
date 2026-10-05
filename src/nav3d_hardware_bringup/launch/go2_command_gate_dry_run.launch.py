from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav3d_hardware_bringup',
            executable='go2_command_gate_dry_run',
            name='go2_command_gate_dry_run',
            output='screen',
            parameters=[{
                'command_topic': '/cmd_vel',
                'enable_topic': '/nav/control_enabled',
                'emergency_stop_topic': '/nav/emergency_stop',
                'publish_rate_hz': 20.0,
                'command_timeout_seconds': 0.15,
                'safety_timeout_seconds': 0.15,
                'max_linear_velocity_m_s': 0.05,
                'max_angular_velocity_rad_s': 0.30,
            }],
        ),
    ])
