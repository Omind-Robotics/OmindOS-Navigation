import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    bringup_share = get_package_share_directory('nav3d_hardware_bringup')
    nmpc_share = get_package_share_directory('nmpc_planner_ros2')
    nmpc_params = os.path.join(nmpc_share, 'config', 'nmpc.yaml')

    pcd_path = LaunchConfiguration('pcd_path')
    map_frame = LaunchConfiguration('map_frame')
    robot_frame = LaunchConfiguration('robot_frame')
    sim_cmd_vel = LaunchConfiguration('sim_cmd_vel_topic')

    simulator = Node(
        package='nav3d_hardware_bringup',
        executable='offline_base_sim_ros2',
        name='offline_base_sim',
        output='screen',
        parameters=[{
            'map_frame': map_frame,
            'base_frame': robot_frame,
            'initial_x': typed('initial_x', float),
            'initial_y': typed('initial_y', float),
            'initial_z': typed('initial_z', float),
            'initial_yaw': typed('initial_yaw', float),
            'initialpose_uses_z': False,
            'motion_mode': LaunchConfiguration('motion_mode'),
            'path_topic': '/path_smooth',
            'cmd_vel_topic': sim_cmd_vel,
            'path_following_speed': typed('path_following_speed', float),
            'align_at_path_end': True,
            'follow_path_height': True,
            'trace_spacing': 0.05,
            'cmd_timeout': 0.35,
            'max_linear_velocity': 0.9,
            'max_angular_velocity': 1.4,
        }],
    )

    planner = Node(
        package='planning_3d_ros2',
        executable='planning_3d_prm_node',
        name='planning_3d_prm',
        output='screen',
        parameters=[{
            'pcd_path': pcd_path,
            'map_frame': map_frame,
            'robot_frame': robot_frame,
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

    local_planner = Node(
        package='nmpc_planner_ros2',
        executable='nmpc_local_planner_node',
        name='nmpc_local_planner',
        output='screen',
        parameters=[nmpc_params],
    )

    # Keep simulated commands physically separate from the real /cmd_vel topic.
    controller = Node(
        package='nmpc_planner_ros2',
        executable='nmpc_controller_node',
        name='nmpc_controller_node',
        output='screen',
        parameters=[nmpc_params, {
            'start_in_auto': typed('start_in_auto', bool),
            'map_frame': map_frame,
            'base_frame': robot_frame,
        }],
        remappings=[('/cmd_vel', sim_cmd_vel)],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='nav3d_offline_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=[
            '-d',
            os.path.join(bringup_share, 'config', 'offline_navigation.rviz'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_path',
            default_value=(
                '/workspace/Nav_3d/src/data/traversable_cloud/'
                'traversable_areas.pcd')),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('motion_mode', default_value='path'),
        DeclareLaunchArgument('path_following_speed', default_value='0.35'),
        DeclareLaunchArgument('start_in_auto', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'sim_cmd_vel_topic', default_value='/sim/cmd_vel'),
        simulator,
        planner,
        smoother,
        local_planner,
        controller,
        rviz,
    ])
