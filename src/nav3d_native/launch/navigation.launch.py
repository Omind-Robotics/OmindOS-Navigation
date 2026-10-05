from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='nav3d_native', executable='planner', output='screen'),
        Node(package='nav3d_native', executable='controller', output='screen'),
    ])
