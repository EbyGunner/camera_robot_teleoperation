"""Launch ONE move_group instance for ONE arm, inside its own namespace.

Replaces the single global move_group from move_group.launch.py. Include
this twice (see robot_main.launch.py) to get::

    /robot_one/move_action     plans manipulator_robot_one
    /robot_two/move_action     plans manipulator_robot_two

Both instances load the FULL dual-arm URDF + SRDF on purpose. Each one
therefore still sees the other arm in its planning scene and collision
checks against it - what is lost is only time synchronisation, i.e. each planner
avoids where the other arm IS, not where it is about to be. With the
bases 1.5 m apart and targets clamped to a +-8 cm box that margin is
ample; if you ever move the bases closer, revisit this.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# The five topics/services rclcpp_action creates under an action name.
_ACTION_ENTITIES = ('send_goal', 'cancel_goal', 'get_result',
                    'feedback', 'status')


def _action_remaps(action_name: str):
    """Remap rules pulling one action out of the node's namespace."""
    return [(f'{action_name}/_action/{entity}',
             f'/{action_name}/_action/{entity}')
            for entity in _ACTION_ENTITIES]


def _controller_remaps(controllers_yaml: Path):
    """Global-controller remaps for every controller listed in the YAML."""
    with controllers_yaml.open() as handle:
        config = yaml.safe_load(handle) or {}
    manager = config.get('moveit_simple_controller_manager', {})

    remaps = []
    for name in manager.get('controller_names', []):
        action_ns = manager.get(name, {}).get('action_ns', '')
        action = f'{name}/{action_ns}' if action_ns else name
        remaps += _action_remaps(action)
    return remaps


def _launch_setup(context, *args, **kwargs):
    arm = LaunchConfiguration('arm').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    controllers_rel = f'config/moveit_controllers_{arm}.yaml'
    controllers_abs = (Path(get_package_share_directory('robot_moveit'))
                       / controllers_rel)
    if not controllers_abs.exists():
        raise RuntimeError(
            f"No MoveIt controller config for arm '{arm}': expected "
            f'{controllers_abs}. Valid arms are the ones with a '
            f'config/moveit_controllers_<arm>.yaml file.')

    moveit_config = (
        MoveItConfigsBuilder('dual_arm_robot', package_name='robot_moveit')
        # Only this arm's controllers, and no controller switching: the two
        # move_group instances share one controller_manager.
        .trajectory_execution(file_path=controllers_rel,
                              moveit_manage_controllers=False)
        .to_moveit_configs()
    )

    # Same set generate_move_group_launch() applies, minus the debug args.
    move_group_configuration = {
        'publish_robot_description': False,
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'capabilities': '',
        'disable_capabilities': '',
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
    }

    remappings = [
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
        ('joint_states', '/joint_states'),
    ] + _controller_remaps(controllers_abs)

    return [Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        namespace=namespace,
        output='screen',
        parameters=[moveit_config.to_dict(), move_group_configuration],
        remappings=remappings,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='robot_one',
            description='Which arm this move_group plans for. Selects '
                        'config/moveit_controllers_<arm>.yaml.'),
        DeclareLaunchArgument(
            'namespace', default_value='robot_one',
            description='ROS namespace for this move_group instance; the '
                        'teleop node talks to /<namespace>/move_action.'),
        OpaqueFunction(function=_launch_setup),
    ])