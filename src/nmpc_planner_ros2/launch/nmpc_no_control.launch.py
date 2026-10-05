import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('nmpc_planner_ros2')
    return LaunchDescription([
        Node(
            package='nmpc_planner_ros2',
            executable='nmpc_local_planner_node',
            name='nmpc_local_planner',
            output='screen',
            parameters=[os.path.join(share, 'config', 'nmpc.yaml')],
        ),
    ])
