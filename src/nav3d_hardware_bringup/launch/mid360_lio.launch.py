import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('nav3d_hardware_bringup')
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'mid360_scan.launch.py')),
        launch_arguments={
            'xfer_format': '1',
            'enable_scan_projection': 'false',
        }.items(),
    )

    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_mapping',
        output='screen',
        parameters=[os.path.join(
            package_share, 'config', 'fast_lio_mid360.yaml')],
        remappings=[
            ('/Odometry', '/lio/odometry_raw'),
            ('/path', '/lio/path'),
            ('/cloud_registered', '/lio/cloud_registered'),
            ('/cloud_registered_body', '/lio/cloud_registered_body'),
            ('/cloud_effected', '/lio/cloud_effected'),
            ('/Laser_map', '/lio/map'),
        ],
    )

    base_odom = Node(
        package='nav3d_hardware_bringup',
        executable='lio_base_odom_adapter',
        name='lio_base_odom_adapter',
        output='screen',
        parameters=[{
            'calibration_file': os.path.join(
                package_share, 'config', 'go2_mid360_extrinsics.yaml'),
        }],
    )

    scan_projection = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='lio_scan_projection',
        output='screen',
        remappings=[
            ('cloud_in', '/lio/cloud_registered_body'),
            ('scan', '/livox/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.02,
            'min_height': -0.18,
            'max_height': 0.30,
            'angle_min': -math.pi,
            'angle_max': math.pi,
            'angle_increment': math.pi / 360.0,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )

    return LaunchDescription([
        sensor_launch,
        fast_lio,
        base_odom,
        scan_projection,
    ])
