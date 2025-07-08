import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    robot_moveit_path = get_package_share_directory('robot_moveit')
    hand_detector_path = get_package_share_directory('hand_tracking')

    move_group_launch_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_moveit_path, 'launch', 'demo.launch.py')
        )
    )

    camera_hand_detector_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hand_detector_path, 'launch', 'hand_tracking.launch.py')
        )
    )

    return LaunchDescription([
        move_group_launch_robot_one,
        camera_hand_detector_node,
    ])