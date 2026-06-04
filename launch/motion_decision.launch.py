# motion_decision.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Args
    ns   = LaunchConfiguration('ns')
    out  = LaunchConfiguration('output')

    enable_turbo_mode = LaunchConfiguration('enable_turbo_mode')
    hz = LaunchConfiguration('hz')
    allowable = LaunchConfiguration('allowable_num_of_not_received_local_path')
    max_velocity = LaunchConfiguration('max_velocity')
    turbo_max_velocity = LaunchConfiguration('turbo_max_velocity')
    max_yawrate = LaunchConfiguration('max_yawrate')

    cmd_vel_topic = LaunchConfiguration('cmd_vel')
    local_path_cmd_vel_topic = LaunchConfiguration('local_path_cmd_vel')  # ← 引数名のみ変更

    node = Node(
        package='motion_decision',
        executable='motion_decision',
        name='motion_decision',
        namespace=ns,
        output=out,               # 'screen' or 'log'
        respawn=True,
        # respawn_delay=2.0,      # 必要なら
        parameters=[{
            'enable_turbo_mode': ParameterValue(enable_turbo_mode, value_type=bool),
            'hz': ParameterValue(hz, value_type=int),
            'allowable_num_of_not_received_local_path': ParameterValue(allowable, value_type=int),
            'max_velocity': ParameterValue(max_velocity, value_type=float),
            'turbo_max_velocity': ParameterValue(turbo_max_velocity, value_type=float),
            'max_yawrate': ParameterValue(max_yawrate, value_type=float),
        }],
        remappings=[
            ('cmd_vel', cmd_vel_topic),
            ('local_path/cmd_vel', local_path_cmd_vel_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='local_planner', description='namespace'),
        DeclareLaunchArgument('output', default_value='screen', description='screen or log'),

        # MotionDecisionParams
        DeclareLaunchArgument('enable_turbo_mode', default_value='false'),
        DeclareLaunchArgument('hz', default_value='20'),
        DeclareLaunchArgument('allowable_num_of_not_received_local_path', default_value='1'),
        DeclareLaunchArgument('max_velocity', default_value='1.0'),
        DeclareLaunchArgument('turbo_max_velocity', default_value='1.66'),
        DeclareLaunchArgument('max_yawrate', default_value='1.0'),

        # Topics
        DeclareLaunchArgument('cmd_vel', default_value='/whill/controller/cmd_vel', description='published cmd_vel topic'),
        DeclareLaunchArgument('local_path_cmd_vel', default_value='local_path/cmd_vel',
                              description='subscribed local path cmd_vel topic'),

        GroupAction([
            PushRosNamespace(ns),
            node,
        ])
    ])
