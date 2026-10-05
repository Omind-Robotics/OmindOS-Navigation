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
                'safe_margin': 0.4,
                'voxel_leaf': 0.4,
                'step_size': 1.0,
                'max_nodes': 10000,
                'k_neigh': 50,
                'max_slope_deg': 45.0,
                'enable_headroom_check': True,
                'auto_replan': True,
                'obstacle_block_radius': 0.0,
                'random_seed': 42,
            }],
        ),
        Node(
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
                'validation_pcd_path': LaunchConfiguration('pcd_path'),
                'path_height_offset': 0.35,
                'max_traversable_distance': 0.45,
            }],
        ),
    ])
