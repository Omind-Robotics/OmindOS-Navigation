import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package_name, launch_file):
    path = os.path.join(
        get_package_share_directory(package_name), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def generate_launch_description():
    map_output = LaunchConfiguration('map_output')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_output',
            default_value='/artifacts/hardware-static-map/static_map',
            description='Output path without extension for the static map',
        ),
        _include('nav3d_hardware_bringup', 'mid360_scan.launch.py'),
        _include('nav3d_hardware_bringup', 'go2_odom_raw.launch.py'),
        Node(
            package='nav3d_native',
            executable='mapper',
            name='nav3d_static_mapper_probe',
            output='screen',
            parameters=[{
                'pose_topic': '/go2/odom_raw',
                'map_output': map_output,
            }],
            remappings=[
                ('/scan', '/livox/scan'),
                ('/map', '/hardware/map'),
            ],
        ),
    ])
