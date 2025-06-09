import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('sereact')
    robot_moveit_path = get_package_share_directory('robot_moveit')
    hand_detector_path = get_package_share_directory('hand_tracking')
    # robot_imitation_path = get_package_share_directory('robot_imitation')

    move_group_launch_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_moveit_path, 'launch', 'move_group.launch.py')
        )
    )

    spawn_controllers_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_moveit_path, 'launch', 'spawn_controllers.launch.py')
        )
    )

    static_virtual_joint_tfs_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_moveit_path, 'launch', 'static_virtual_joint_tfs.launch.py')
        )
    )

    rsp_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_moveit_path, 'launch', 'rsp.launch.py')
        )
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(robot_moveit_path, "config", "ros2_controllers.yaml")],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    camera_hand_detector_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hand_detector_path, 'launch', 'hand_tracking.launch.py')
        )
    )

    # robot_imitation_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(robot_imitation_path, 'launch', 'robot_imitation.launch.py')
    #     )
    # )

    rviz_config_file = os.path.join(package_path, 'src', 'rviz', 'robot_view.rviz')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        static_virtual_joint_tfs_robot_one,
        rsp_robot_one,
        ros2_control_node,
        spawn_controllers_robot_one,
        move_group_launch_robot_one,
        rviz_node,
        camera_hand_detector_node,
        # robot_imitation_node,
    ])