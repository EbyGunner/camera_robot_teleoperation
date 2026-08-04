import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    robot_moveit_path = Path(get_package_share_directory('robot_moveit'))
    hand_detector_path = Path(get_package_share_directory('hand_tracking'))
    robot_imitation_path = Path(get_package_share_directory('robot_imitation'))

    position_scale_arg = DeclareLaunchArgument(
        'position_scale',
        default_value='0.50',
        description='Scale factor mapping hand motion to robot motion'
        )
    
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = robot_moveit_path / "launch/static_virtual_joint_tfs.launch.py"

    moveit_virtual_joints = None
    if virtual_joints_launch.exists():
        moveit_virtual_joints = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(virtual_joints_launch)),
        )
        
    # Given the published joint states, publish tf for the robot links
    moveit_joinst_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(robot_moveit_path / "launch/rsp.launch.py")
        ),
    )

    moveit_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(robot_moveit_path / "launch/move_group.launch.py")
        ),
    )

    # Fake joint driver 
    moveit_robotdesc_launch = Node( 
        package="controller_manager", executable="ros2_control_node", 
        parameters=[robot_moveit_path / "config/ros2_controllers.yaml", ], 
        remappings=[ ("/controller_manager/robot_description", "/robot_description"), ], 
    )

    moveit_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(robot_moveit_path / "launch/spawn_controllers.launch.py")
        ),
    )

    camera_hand_detector_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(hand_detector_path / "launch/hand_tracking.launch.py")
        )
    )

    robot_imitation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(robot_imitation_path / "launch/robot_imitation.launch.py")
        ),
        launch_arguments={
            'position_scale': LaunchConfiguration('position_scale')
        }.items()
    )

    rviz_config = os.path.join(get_package_share_directory('robot_main'),
        'src', 
        'rviz', 
        'robot_view.rviz'
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    rviz_delayed = TimerAction(
        period=6.0,
        actions=[rviz_node]
    )
    
    launch_items = [
        position_scale_arg,
        moveit_joinst_state_launch,
        moveit_group_launch,
        moveit_robotdesc_launch,
        moveit_controllers_launch,
        camera_hand_detector_node,
        robot_imitation_node,
        rviz_delayed,
    ]

    # Only add if it exists
    if moveit_virtual_joints:
        launch_items.append(moveit_virtual_joints)

    return LaunchDescription(launch_items)