from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapping_seconds = LaunchConfiguration('mapping_phase_seconds')
    return LaunchDescription([
        DeclareLaunchArgument('mapping_phase_seconds', default_value='5.0'),
        DeclareLaunchArgument('map_output', default_value='/tmp/nav3d_demo_map'),
        DeclareLaunchArgument('result_file', default_value='/tmp/nav3d_demo_result.json'),
        Node(package='nav3d_native', executable='simulator', output='screen',
             parameters=[{'mapping_phase_seconds': mapping_seconds}]),
        Node(package='nav3d_native', executable='mapper', output='screen',
             parameters=[{'map_output': LaunchConfiguration('map_output')}]),
        TimerAction(period=mapping_seconds, actions=[
            Node(package='nav3d_native', executable='localizer', output='screen'),
            Node(package='nav3d_native', executable='planner', output='screen'),
            Node(package='nav3d_native', executable='controller', output='screen'),
        ]),
        Node(package='nav3d_native', executable='demo_supervisor', output='screen',
             parameters=[{
                 'mapping_phase_seconds': mapping_seconds,
                 'result_file': LaunchConfiguration('result_file'),
             }]),
    ])
