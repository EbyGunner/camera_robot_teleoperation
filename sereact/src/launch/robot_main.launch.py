import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('sereact')
    robot_1_moveit_path = get_package_share_directory('robot_moveit')

    move_group_launch_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_1_moveit_path, 'launch', 'move_group.launch.py')
        )
    )

    spawn_controllers_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_1_moveit_path, 'launch', 'spawn_controllers.launch.py')
        )
    )

    static_virtual_joint_tfs_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_1_moveit_path, 'launch', 'static_virtual_joint_tfs.launch.py')
        )
    )

    rsp_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_1_moveit_path, 'launch', 'rsp.launch.py')
        )
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(robot_1_moveit_path, "config", "ros2_controllers.yaml")],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

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
    ])
