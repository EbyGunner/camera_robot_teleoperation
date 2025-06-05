from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hand_tracking',
            executable='hand_tracking_node',
            name='hand_tracking_node',
            output='screen',
            emulate_tty=True,
        )
    ])
