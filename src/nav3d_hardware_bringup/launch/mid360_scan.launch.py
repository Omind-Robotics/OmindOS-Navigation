import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _finite_vector(document, key, length, label):
    value = document.get(key)
    if not isinstance(value, list) or len(value) != length:
        raise RuntimeError(f'{label}.{key} must contain {length} values')
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise RuntimeError(f'{label}.{key} contains a non-finite value')
    return result


def _load_extrinsics(package_share):
    path = os.path.join(
        package_share, 'config', 'go2_mid360_extrinsics.yaml')
    with open(path, encoding='utf-8') as stream:
        calibration = yaml.safe_load(stream)
    if not isinstance(calibration, dict):
        raise RuntimeError(f'invalid calibration document: {path}')
    if calibration.get('runtime_allowed') is not True:
        raise RuntimeError(f'calibration is not runtime-approved: {path}')
    transforms = calibration.get('transforms')
    if not isinstance(transforms, dict):
        raise RuntimeError(f'calibration transforms missing: {path}')

    expected = {
        'base_link_to_body': ('base_link', 'body'),
        'body_to_lidar': ('body', 'livox_frame'),
        'lidar_to_imu': ('livox_frame', 'imu_link'),
    }
    result = []
    for name, frames in expected.items():
        transform = transforms.get(name)
        if not isinstance(transform, dict):
            raise RuntimeError(f'calibration transform missing: {name}')
        if (transform.get('parent'), transform.get('child')) != frames:
            raise RuntimeError(f'unexpected frame pair for {name}')
        translation = _finite_vector(
            transform, 'translation_m', 3, name)
        quaternion = _finite_vector(
            transform, 'rotation_xyzw', 4, name)
        norm = math.sqrt(sum(item * item for item in quaternion))
        if abs(norm - 1.0) > 1.0e-4:
            raise RuntimeError(f'non-unit quaternion for {name}: {norm}')
        quaternion = [item / norm for item in quaternion]
        result.append((name, frames, translation, quaternion))
    return result


def _static_transform_node(name, frames, translation, quaternion):
    parent, child = frames
    x, y, z = translation
    qx, qy, qz, qw = quaternion
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--qx', str(qx), '--qy', str(qy), '--qz', str(qz),
            '--qw', str(qw), '--frame-id', parent,
            '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    xfer_format = LaunchConfiguration('xfer_format')
    enable_scan_projection = LaunchConfiguration('enable_scan_projection')
    package_share = get_package_share_directory('nav3d_hardware_bringup')
    livox_config = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config',
        'MID360_config.json',
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': xfer_format,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': livox_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
        remappings=[
            # The vendor packet reports acceleration in g. Preserve that raw
            # stream separately and expose a ROS SI-unit stream via the
            # adapter below.
            ('/livox/imu', '/livox/imu_raw'),
        ],
    )

    imu_adapter = Node(
        package='nav3d_hardware_bringup',
        executable='livox_imu_adapter',
        name='livox_imu_adapter',
        output='screen',
        parameters=[{
            'input_topic': '/livox/imu_raw',
            'output_topic': '/livox/imu',
        }],
    )

    scan_projection = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_scan_projection',
        output='screen',
        condition=IfCondition(enable_scan_projection),
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/livox/scan'),
        ],
        parameters=[{
            # Apply the accepted Go2/MID360 mounting transform before taking
            # the planar slice. The output remains isolated from /scan.
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            # In base_link, the accepted standing floor is about -0.23 m.
            # This slice starts roughly 5 cm above it to reject ground while
            # retaining low obstacles.
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

    static_transforms = [
        _static_transform_node(*transform)
        for transform in _load_extrinsics(package_share)
    ]
    return LaunchDescription([
        DeclareLaunchArgument('xfer_format', default_value='0'),
        DeclareLaunchArgument(
            'enable_scan_projection', default_value='true'),
    ] + static_transforms + [livox_driver, imu_adapter, scan_projection])
