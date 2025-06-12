from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planning_time',
            default_value='5.0',
            description='Allowed planning time in seconds'
        ),
        DeclareLaunchArgument(
            'velocity_scaling',
            default_value='0.5',
            description='Velocity scaling factor (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'acceleration_scaling',
            default_value='0.5',
            description='Acceleration scaling factor (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='world',
            description='World frame for TF transformations'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug mode'
        ),

        Node(
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
                'debug': LaunchConfiguration('debug')
            }]
        )
    ])
