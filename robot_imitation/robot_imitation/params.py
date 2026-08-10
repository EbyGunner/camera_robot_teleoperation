"""Central parameter handling for the imitation node.

Every tunable lives here so the rest of the code never calls
``declare_parameter`` / ``get_parameter`` directly.

Changed for the two-move_group layout: ``combined_group`` and
``merge_window`` are gone (there is no combined goal and nothing to merge
any more) and each arm gained a ``*_namespace`` parameter naming the
move_group instance that plans for it.
"""

from dataclasses import dataclass, fields


@dataclass
class ImitationParams:
    # --- robots -----------------------------------------------------------
    robot1_name: str = 'robot_one'      # controlled by the LEFT hand
    robot2_name: str = 'robot_two'      # controlled by the RIGHT hand

    # Namespace of the move_group instance planning for each arm; the
    # action the node talks to is /<namespace>/move_action. Must match the
    # 'namespace' launch argument given to move_group_ns.launch.py.
    robot1_namespace: str = 'robot_one'
    robot2_namespace: str = 'robot_two'

    # Namespace of the ros2_control controllers. Empty = global, which is
    # how robot_main.launch.py ships: one controller_manager drives the
    # mock hardware for all 12 joints. Only set this if you later split
    # the controller_manager per arm as well.
    controller_namespace: str = ''

    # --- behaviour --------------------------------------------------------
    debug: bool = False
    # '' = follow the hands. 'exact_current_pose' / 'fixed_offset' are
    # planning diagnostics only: they command the current pose (no motion)
    # or a fixed 1 cm sideways step.
    diagnostic_mode: str = ''

    # --- hand -> target mapping (signed gains, metres per unit hand input)
    # hand.x (horizontal) -> robot y, hand.y (vertical) -> robot z,
    # hand.z (depth from hand size) -> robot x.
    # scale_x = 0.0 keeps x frozen (monocular depth is too noisy).
    scale_x: float = 0.0
    scale_y: float = -0.30
    scale_z: float = -0.30
    # Targets are clamped to +-workspace_extent around the reference pose.
    workspace_extent: float = 0.08

    # --- re-planning gates ------------------------------------------------
    position_threshold: float = 0.02     # m
    orientation_threshold: float = 0.2   # rad
    # Per arm, and now genuinely per arm: with one move_group per arm this
    # is the only thing limiting how often an arm re-plans, so it is worth
    # tuning down (0.4-0.6 s) once you trust the setup. It used to also
    # have to absorb the other arm's planning time.
    plan_cooldown: float = 1.0           # s
    max_missing_frames: int = 5          # hand lost -> reset reference

    # --- MoveIt request ---------------------------------------------------
    planning_time: float = 5.0
    num_planning_attempts: int = 4
    velocity_scaling: float = 0.5
    acceleration_scaling: float = 0.5
    planner_id: str = ''                 # '' = pipeline default (RRTConnect)
    position_tolerance: float = 0.02     # goal sphere radius (m)
    use_orientation_constraint: bool = True
    orientation_tolerance: float = 0.5   # rad per axis
    goal_timeout: float = 20.0           # s before a stuck goal is cancelled

    # --- gripper ----------------------------------------------------------
    gripper_open_position: float = 0.0
    gripper_closed_position: float = 0.06
    gripper_effort: float = 0.5

    # ---------------------------------------------------------------------
    @classmethod
    def from_node(cls, node) -> 'ImitationParams':
        """Declare every field on *node* and return the resolved values."""
        values = {}
        for f in fields(cls):
            node.declare_parameter(f.name, getattr(cls, f.name, f.default))
            values[f.name] = node.get_parameter(f.name).value
        return cls(**values)

    # ---------------------------------------------------------------------
    @property
    def robots(self) -> tuple:
        return (self.robot1_name, self.robot2_name)

    def _suffix(self, robot: str) -> str:
        return '_1' if robot == self.robot1_name else '_2'

    # frame helpers -------------------------------------------------------
    def base_frame(self, robot: str) -> str:
        return f'base_link_robot{self._suffix(robot)}'

    def tip_link(self, robot: str) -> str:
        return f'link6_robot{self._suffix(robot)}'

    def planning_group(self, robot: str) -> str:
        return f'manipulator_{robot}'

    # action helpers ------------------------------------------------------
    def namespace(self, robot: str) -> str:
        ns = (self.robot1_namespace if robot == self.robot1_name
              else self.robot2_namespace).strip('/')
        return ns

    def move_action(self, robot: str) -> str:
        """MoveGroup action of the move_group instance owning *robot*."""
        ns = self.namespace(robot)
        return f'/{ns}/move_action' if ns else '/move_action'

    def gripper_action(self, robot: str) -> str:
        ns = self.controller_namespace.strip('/')
        prefix = f'/{ns}' if ns else ''
        return f'{prefix}/gripper_{robot}_controller/gripper_cmd'