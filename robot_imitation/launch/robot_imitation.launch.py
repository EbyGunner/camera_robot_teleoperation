from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

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
        
        Node(
            package='robot_imitation',
            executable='imitation_algorithm',
            name='hand_to_robot_controller',
            parameters=[{
                'robot1_name': 'robot_one',
                'robot2_name': 'robot_two',
                'planning_time': 5.0,
                'velocity_scaling': 0.5,
                'acceleration_scaling': 0.5
            }]
        )
    ])