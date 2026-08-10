"""Top-level bring-up: one robot, two independently-planning arms.

Layout (changed from the single-move_group version):

    global namespace
      robot_state_publisher      full dual-arm URDF -> /tf
      static virtual joint TFs   world -> base_link
      ros2_control_node          ONE controller_manager, all 4 controllers
      joint_state_broadcaster    -> /joint_states
      rviz2

    /robot_one
      move_group                 plans manipulator_robot_one

    /robot_two
      move_group                 plans manipulator_robot_two

    hand_to_robot_controller     one node, two MoveGroup clients
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_moveit_path = Path(get_package_share_directory('robot_moveit'))
    hand_detector_path = Path(get_package_share_directory('hand_tracking'))
    robot_imitation_path = Path(get_package_share_directory('robot_imitation'))

    # ---------------------------------------------------------------- args
    args = [
        DeclareLaunchArgument(
            'robot1_namespace', default_value='robot_one',
            description='Namespace of the move_group planning for arm 1 '
                        '(the LEFT hand drives this arm).'),
        DeclareLaunchArgument(
            'robot2_namespace', default_value='robot_two',
            description='Namespace of the move_group planning for arm 2.'),
        DeclareLaunchArgument('planning_time', default_value='5.0',
                              description='Allowed planning time in seconds'),
        DeclareLaunchArgument('velocity_scaling', default_value='0.5',
                              description='Velocity scaling factor (0.0-1.0)'),
        DeclareLaunchArgument('acceleration_scaling', default_value='0.5',
                              description='Acceleration scaling (0.0-1.0)'),
        DeclareLaunchArgument('workspace_extent', default_value='0.08',
                              description='Half-size (m) of the teleop box'),
        DeclareLaunchArgument('plan_cooldown', default_value='1.0',
                              description='Minimum seconds between re-plans '
                                          'for one arm'),
        DeclareLaunchArgument('use_orientation_constraint',
                              default_value='true',
                              description='Set false for position-only IK'),
        DeclareLaunchArgument('diagnostic_mode', default_value='',
                              description="'' = follow hands; "
                                          "'exact_current_pose' or "
                                          "'fixed_offset' to debug planning"),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Verbose logging'),
    ]

    # ------------------------------------------------------- global layer
    # Static TF for the SRDF virtual joint (world -> base_link), if present.
    virtual_joints_launch = (robot_moveit_path
                             / 'launch/static_virtual_joint_tfs.launch.py')
    moveit_virtual_joints = None
    if virtual_joints_launch.exists():
        moveit_virtual_joints = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(virtual_joints_launch)))

    # One robot_state_publisher for the whole 12-DOF model: both namespaced
    # move_group instances and RViz read the same /tf tree.
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(robot_moveit_path / 'launch/rsp.launch.py')))

    # One controller_manager (mock hardware covers all 12 joints).
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_moveit_path / 'config/ros2_controllers.yaml'],
        remappings=[('/controller_manager/robot_description',
                     '/robot_description')],
        output='screen',
    )

    # Spawns all four controllers plus joint_state_broadcaster, driven by
    # the untouched config/moveit_controllers.yaml.
    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(robot_moveit_path / 'launch/spawn_controllers.launch.py')))

    # --------------------------------------------------- per-arm move_group
    move_group_ns_launch = str(robot_moveit_path
                               / 'launch/move_group_ns.launch.py')

    move_group_arm1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_ns_launch),
        launch_arguments={
            'arm': 'robot_one',
            'namespace': LaunchConfiguration('robot1_namespace'),
        }.items())

    move_group_arm2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_ns_launch),
        launch_arguments={
            'arm': 'robot_two',
            'namespace': LaunchConfiguration('robot2_namespace'),
        }.items())

    # Give RSP and the controller spawners a moment: move_group logs a wall
    # of "waiting for current robot state" if it starts before /joint_states.
    move_groups_delayed = TimerAction(
        period=3.0, actions=[move_group_arm1, move_group_arm2])

    # ------------------------------------------------------------- teleop
    hand_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(hand_detector_path / 'launch/hand_tracking.launch.py')))

    imitation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(robot_imitation_path / 'launch/robot_imitation.launch.py')),
        launch_arguments={
            'robot1_namespace': LaunchConfiguration('robot1_namespace'),
            'robot2_namespace': LaunchConfiguration('robot2_namespace'),
            'planning_time': LaunchConfiguration('planning_time'),
            'velocity_scaling': LaunchConfiguration('velocity_scaling'),
            'acceleration_scaling': LaunchConfiguration(
                'acceleration_scaling'),
            'workspace_extent': LaunchConfiguration('workspace_extent'),
            'plan_cooldown': LaunchConfiguration('plan_cooldown'),
            'use_orientation_constraint': LaunchConfiguration(
                'use_orientation_constraint'),
            'diagnostic_mode': LaunchConfiguration('diagnostic_mode'),
            'debug': LaunchConfiguration('debug'),
        }.items())

    # --------------------------------------------------------------- rviz
    rviz_config = os.path.join(get_package_share_directory('robot_main'),
                               'src', 'rviz', 'robot_view.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen')
    rviz_delayed = TimerAction(period=6.0, actions=[rviz_node])

    launch_items = args + [
        rsp_launch,
        ros2_control_node,
        spawn_controllers,
        move_groups_delayed,
        hand_detector,
        imitation_node,
        rviz_delayed,
    ]
    if moveit_virtual_joints:
        launch_items.append(moveit_virtual_joints)

    return LaunchDescription(launch_items)