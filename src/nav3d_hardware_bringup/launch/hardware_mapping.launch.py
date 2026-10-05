import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('nav3d_hardware_bringup')
    map_output = LaunchConfiguration('map_output')
    sensor_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'mid360_lio.launch.py')))
    mapper = Node(
        package='nav3d_native',
        executable='mapper',
        name='nav3d_hardware_mapper',
        output='screen',
        parameters=[{
            'pose_topic': '/lio/odom',
            'map_output': map_output,
        }],
        remappings=[
            ('/scan', '/livox/scan'),
            ('/map', '/hardware/map'),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_output',
            default_value='/artifacts/hardware-lio-map/map',
            description='Output path without extension for PGM/YAML map'),
        sensor_lio,
        mapper,
    ])
