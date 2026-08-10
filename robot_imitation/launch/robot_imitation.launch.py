from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # Must match the 'namespace' argument each move_group was launched
        # with; the node talks to /<namespace>/move_action.
        DeclareLaunchArgument('robot1_namespace', default_value='robot_one',
                              description='move_group namespace for arm 1 '
                                          '(driven by the LEFT hand)'),
        DeclareLaunchArgument('robot2_namespace', default_value='robot_two',
                              description='move_group namespace for arm 2 '
                                          '(driven by the RIGHT hand)'),
        DeclareLaunchArgument('controller_namespace', default_value='',
                              description='Namespace of the ros2_control '
                                          'controllers; empty = global, '
                                          'which is how robot_main ships'),
        DeclareLaunchArgument('planning_time', default_value='5.0',
                              description='Allowed planning time in seconds'),
        DeclareLaunchArgument('velocity_scaling', default_value='0.5',
                              description='Velocity scaling factor (0.0-1.0)'),
        DeclareLaunchArgument('acceleration_scaling', default_value='0.5',
                              description='Acceleration scaling (0.0-1.0)'),
        DeclareLaunchArgument('plan_cooldown', default_value='1.0',
                              description='Minimum seconds between re-plans '
                                          'for one arm. Now purely per-arm: '
                                          'lower it (0.4-0.6) for snappier '
                                          'tracking once the setup is stable'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Enable debug logging'),
        DeclareLaunchArgument(
            'diagnostic_mode', default_value='',
            description="'' = follow hands; 'exact_current_pose' or "
                        "'fixed_offset' for planning diagnostics"),
        DeclareLaunchArgument(
            'use_orientation_constraint', default_value='true',
            description='Set false when running position_only_ik on the '
                        'unpatched URDF'),
        DeclareLaunchArgument('workspace_extent', default_value='0.08',
                              description='Half-size (m) of the teleop box '
                                          'around the reference pose'),
    ]

    node = Node(
        package='robot_imitation',
        executable='imitation_algorithm',
        name='hand_to_robot_controller',
        output='screen',
        parameters=[{
            'robot1_name': 'robot_one',
            'robot2_name': 'robot_two',
            'robot1_namespace': LaunchConfiguration('robot1_namespace'),
            'robot2_namespace': LaunchConfiguration('robot2_namespace'),
            'controller_namespace':
                LaunchConfiguration('controller_namespace'),
            'planning_time': LaunchConfiguration('planning_time'),
            'velocity_scaling': LaunchConfiguration('velocity_scaling'),
            'acceleration_scaling': LaunchConfiguration('acceleration_scaling'),
            'plan_cooldown': LaunchConfiguration('plan_cooldown'),
            'debug': LaunchConfiguration('debug'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'use_orientation_constraint':
                LaunchConfiguration('use_orientation_constraint'),
            'workspace_extent': LaunchConfiguration('workspace_extent'),
        }],
    )

    return LaunchDescription(args + [node])