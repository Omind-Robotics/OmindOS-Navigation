"""One navigation core, selectable platform profile, simulation output only."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav3d_native.platform_guard import load_profile


def launch_nodes(context):
    platform = LaunchConfiguration('platform').perform(context)
    profile_file = LaunchConfiguration('profile_file').perform(context)
    profile = load_profile(platform, profile_file or None)  # Fail before any node starts.
    root = '/omindos_preview'
    common = [(x, root + x) for x in (
        '/scan', '/map', '/ground_truth/odom', '/localization/pose',
        '/plan', '/goal_pose', '/nav3d/demo_result', '/tf', '/tf_static')]
    common += [('/wheel/odom', root + '/odom')]
    mapping = 5.0

    def node(executable, parameters=None, remappings=None):
        return Node(package='nav3d_native', executable=executable, output='screen',
                    parameters=parameters or [], remappings=common + (remappings or []))

    return [
        node('simulator', [{'mapping_phase_seconds': mapping,
                            'lateral_velocity_enabled': profile.max_vy > 0,
                            'command_timeout_seconds': 0.3}],
             [('/cmd_vel', root + '/robot/cmd_vel')]),
        node('mapper', [{'map_output': LaunchConfiguration('map_output')}]),
        TimerAction(period=mapping, actions=[
            node('localizer'), node('planner'),
            node('controller', remappings=[('/cmd_vel', root + '/navigation/cmd_vel')]),
        ]),
        node('platform_adapter', [{'platform': platform, 'profile_file': profile_file,
                                   'enable_preview_motion': True}]),
        node('demo_supervisor', [{'mapping_phase_seconds': mapping,
                                  'timeout_seconds': 100.0, 'require_stopped': True,
                                  'result_file': LaunchConfiguration('result_file')}],
             [('/cmd_vel', root + '/robot/cmd_vel')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('platform', default_value='wheeled_diff'),
        DeclareLaunchArgument('profile_file', default_value=''),
        DeclareLaunchArgument('result_file', default_value='/tmp/omindos_platform_result.json'),
        DeclareLaunchArgument('map_output', default_value='/tmp/omindos_platform_map'),
        OpaqueFunction(function=launch_nodes),
    ])
