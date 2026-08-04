from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('planning_time', default_value='5.0',
                              description='Allowed planning time in seconds'),
        DeclareLaunchArgument('velocity_scaling', default_value='0.5',
                              description='Velocity scaling factor (0.0-1.0)'),
        DeclareLaunchArgument('acceleration_scaling', default_value='0.5',
                              description='Acceleration scaling factor (0.0-1.0)'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Enable debug logging'),
        DeclareLaunchArgument(
            'diagnostic_mode', default_value='',
            description="'' = follow hands; 'exact_current_pose' or "
                        "'fixed_offset' for planning diagnostics"),
        DeclareLaunchArgument(
            'use_orientation_constraint', default_value='true',
            description='Set false when running position_only_ik on the '
                        'unpatched URDF (see PATCH_NOTES.md)'),
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
            'planning_time': LaunchConfiguration('planning_time'),
            'velocity_scaling': LaunchConfiguration('velocity_scaling'),
            'acceleration_scaling': LaunchConfiguration('acceleration_scaling'),
            'debug': LaunchConfiguration('debug'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'use_orientation_constraint':
                LaunchConfiguration('use_orientation_constraint'),
            'workspace_extent': LaunchConfiguration('workspace_extent'),
        }],
    )

    return LaunchDescription(args + [node])
