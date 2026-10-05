import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hardware_share = get_package_share_directory('nav3d_hardware_bringup')
    nmpc_share = get_package_share_directory('nmpc_planner_ros2')

    start_lio = LaunchConfiguration('start_lio')
    start_controller = LaunchConfiguration('start_controller')
    start_in_auto = LaunchConfiguration('start_in_auto')
    start_go2_bridge = LaunchConfiguration('start_go2_bridge')
    start_rviz = LaunchConfiguration('start_rviz')
    pcd_path = LaunchConfiguration('pcd_path')

    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            hardware_share, 'launch', 'mid360_lio.launch.py')),
        condition=IfCondition(start_lio),
    )

    # This is the ROS 2 equivalent of the original ROS 1
    # map -> camera_init static transform. It is not global localization:
    # the robot must start at the mapping start position and heading.
    map_to_lio_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_lio_odom_known_start',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('map_to_lio_x'),
            '--y', LaunchConfiguration('map_to_lio_y'),
            '--z', LaunchConfiguration('map_to_lio_z'),
            '--qx', LaunchConfiguration('map_to_lio_qx'),
            '--qy', LaunchConfiguration('map_to_lio_qy'),
            '--qz', LaunchConfiguration('map_to_lio_qz'),
            '--qw', LaunchConfiguration('map_to_lio_qw'),
            '--frame-id', 'map',
            '--child-frame-id', 'lio_odom',
        ],
    )

    planner = Node(
        package='planning_3d_ros2',
        executable='planning_3d_prm_node',
        name='planning_3d_prm',
        output='screen',
        parameters=[{
            'pcd_path': pcd_path,
            'map_frame': 'map',
            'robot_frame': 'base_link',
            'edt_xy_expand': 1.0,
            'edt_z_thickness': 3,
            'robot_height': 1.0,
            'enable_headroom_check': True,
            'safe_margin': 0.4,
            'voxel_leaf': 0.4,
            'use_clearance_penalty': True,
            'clearance_penalty_weight': 22750.0,
            'clearance_penalty_scale': 0.3,
            'use_soft_penalty': False,
            'step_size': 1.0,
            'max_nodes': 10000,
            'k_neigh': 50,
            'max_slope_deg': 45.0,
            'auto_replan': True,
            'replan_frequency': 1.0,
            'pos_tolerance': 0.5,
            'pos_tolerance_exit': 2.0,
            'yaw_tolerance': 0.14,
            'obstacle_block_radius': 0.0,
            'random_seed': 42,
        }],
    )

    smoother = Node(
        package='planning_3d_ros2',
        executable='bezier_path_optimizer_node',
        name='bezier_path_optimizer',
        output='screen',
        parameters=[{
            'sample_rate': 20,
            'q1_scale': 0.3,
            'q1_max_ratio': 1.0,
            'enable_orientation': True,
            'end_strategy': 'keep_original',
            'validation_pcd_path': pcd_path,
            'path_height_offset': 0.35,
            'max_traversable_distance': 0.45,
        }],
    )

    nmpc_params = os.path.join(nmpc_share, 'config', 'nmpc.yaml')
    local_planner = Node(
        package='nmpc_planner_ros2',
        executable='nmpc_local_planner_node',
        name='nmpc_local_planner',
        output='screen',
        parameters=[nmpc_params, {
            'min_angular_vel': ParameterValue(
                LaunchConfiguration('max_angular_velocity'), value_type=float),
            'goal_align_angular_vel': ParameterValue(
                LaunchConfiguration('max_angular_velocity'), value_type=float),
            'mpc.max_linear_vel': ParameterValue(
                LaunchConfiguration('max_linear_velocity'), value_type=float),
            'mpc.max_angular_vel': ParameterValue(
                LaunchConfiguration('max_angular_velocity'), value_type=float),
            'mpc.fallback_max_linear_vel': ParameterValue(
                LaunchConfiguration('max_linear_velocity'), value_type=float),
            'mpc.fallback_max_angular_vel': ParameterValue(
                LaunchConfiguration('max_angular_velocity'), value_type=float),
        }],
    )
    controller = Node(
        package='nmpc_planner_ros2',
        executable='nmpc_controller_node',
        name='nmpc_controller_node',
        output='screen',
        condition=IfCondition(start_controller),
        parameters=[nmpc_params, {
            'start_in_auto': ParameterValue(start_in_auto, value_type=bool),
        }],
    )

    go2_bridge = Node(
        package='nav3d_hardware_bringup',
        executable='go2_base_controller_ros2',
        name='go2_base_controller_ros2',
        output='screen',
        condition=IfCondition(start_go2_bridge),
        parameters=[{
            'unitree_interface': LaunchConfiguration('unitree_interface'),
            'unitree_peer': LaunchConfiguration('unitree_peer'),
            'unitree_domain_id': ParameterValue(
                LaunchConfiguration('unitree_domain_id'), value_type=int),
            'allow_multicast': 'spdp',
            'command_timeout_seconds': ParameterValue(
                LaunchConfiguration('command_timeout_seconds'),
                value_type=float),
            'control_frequency': 50.0,
            'sport_client_timeout_seconds': 10.0,
            'max_linear_velocity': ParameterValue(
                LaunchConfiguration('max_linear_velocity'), value_type=float),
            'max_lateral_velocity': 0.0,
            'max_angular_velocity': ParameterValue(
                LaunchConfiguration('max_angular_velocity'), value_type=float),
            'min_effective_linear_velocity': ParameterValue(
                LaunchConfiguration('min_effective_linear_velocity'),
                value_type=float),
            'min_effective_angular_velocity': ParameterValue(
                LaunchConfiguration('min_effective_angular_velocity'),
                value_type=float),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='nav3d_hardware_rviz',
        output='screen',
        condition=IfCondition(start_rviz),
        arguments=[
            '-d',
            os.path.join(hardware_share, 'config', 'hardware_navigation.rviz'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_lio', default_value='true'),
        DeclareLaunchArgument('start_controller', default_value='false'),
        DeclareLaunchArgument('start_in_auto', default_value='false'),
        DeclareLaunchArgument('start_go2_bridge', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('unitree_interface', default_value='enp86s0'),
        DeclareLaunchArgument(
            'unitree_peer', default_value='192.168.123.161'),
        DeclareLaunchArgument('unitree_domain_id', default_value='0'),
        DeclareLaunchArgument(
            'command_timeout_seconds', default_value='0.25'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.40'),
        DeclareLaunchArgument('max_angular_velocity', default_value='0.40'),
        DeclareLaunchArgument(
            'min_effective_linear_velocity', default_value='0.0'),
        DeclareLaunchArgument(
            'min_effective_angular_velocity', default_value='0.0'),
        DeclareLaunchArgument(
            'pcd_path',
            default_value=(
                '/workspace/Nav_3d/src/data/traversable_cloud/'
                'traversable_areas_start_patch.pcd')),
        DeclareLaunchArgument('map_to_lio_x', default_value='0.0'),
        DeclareLaunchArgument('map_to_lio_y', default_value='0.0'),
        DeclareLaunchArgument('map_to_lio_z', default_value='0.0'),
        DeclareLaunchArgument('map_to_lio_qx', default_value='-0.030724274'),
        DeclareLaunchArgument('map_to_lio_qy', default_value='0.088034341'),
        DeclareLaunchArgument('map_to_lio_qz', default_value='0.0'),
        DeclareLaunchArgument('map_to_lio_qw', default_value='0.995643497'),
        lio,
        map_to_lio_odom,
        planner,
        smoother,
        local_planner,
        controller,
        go2_bridge,
        rviz,
    ])
