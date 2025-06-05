import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('sereact')

    print("here1")

    moveit_config_dir_robot_1 = os.path.join(package_path, 'src', 'moveit_robot_1')
    # moveit_config_dir_robot_2 = os.path.join(package_path, 'src', 'moveit_robot_2')

    move_group_launch_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir_robot_1, 'launch', 'move_group.launch.py')
        )
    )

    spawn_controllers_robot_one = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir_robot_1, 'launch', 'spawn_controllers.launch.py')
        )
    )

    # move_group_launch_robot_two = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(moveit_config_dir_robot_2, 'launch', 'move_group.launch.py')
    #     )
    # )

    # spawn_controllers_robot_two = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(moveit_config_dir_robot_2, 'launch', 'spawn_controllers.launch.py')
    #     )
    # )

    # static_tf_robot1 = Node(
    # package="tf2_ros",
    # executable="static_transform_publisher",
    # name="robot1_tf_pub",
    # arguments=["0", "-0.75", "0", "0", "0", "0", "world", "base_link_robot_1"],
    # namespace='robot_1'
    # )

    # static_tf_robot2 = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="robot2_tf_pub",
    #     arguments=["0", "0.75", "0", "0", "0", "0", "world", "base_link_robot_2"],
    #     namespace='robot_2'
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
        move_group_launch_robot_one,
        spawn_controllers_robot_one,
        # move_group_launch_robot_two,
        # spawn_controllers_robot_two,
        # static_tf_robot1,
        # static_tf_robot2,
        rviz_node,
    ])
