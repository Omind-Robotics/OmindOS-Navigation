from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_path',
            default_value=(
                '/workspace/Nav_3d/src/data/traversable_cloud/'
                'traversable_areas.pcd')),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        Node(
            package='planning_3d_ros2',
            executable='planning_3d_prm_node',
            name='planning_3d_prm',
            output='screen',
            parameters=[{
                'pcd_path': LaunchConfiguration('pcd_path'),
                'map_frame': LaunchConfiguration('map_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
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
                'obstacle_timeout': 5.0,
                'random_seed': 42,
            }],
        ),
    ])
