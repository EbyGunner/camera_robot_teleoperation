"""Central parameter handling for the imitation node.

Every tunable lives here so the rest of the code never calls
``declare_parameter`` / ``get_parameter`` directly.
"""

from dataclasses import dataclass, fields


@dataclass
class ImitationParams:
    # --- robots -----------------------------------------------------------
    robot1_name: str = 'robot_one'      # controlled by the LEFT hand
    robot2_name: str = 'robot_two'      # controlled by the RIGHT hand

    # --- behaviour --------------------------------------------------------
    debug: bool = False
    # '' = follow the hands. 'exact_current_pose' / 'fixed_offset' are
    # planning diagnostics only: they command the current pose (no motion)
    # or a fixed 1 cm sideways step. The old code defaulted to
    # 'exact_current_pose', which is one of the reasons the robot never moved.
    diagnostic_mode: str = ''

    # --- hand -> target mapping (signed gains, metres per unit hand input)
    # hand.x (horizontal) -> robot y, hand.y (vertical) -> robot z,
    # hand.z (depth from hand size) -> robot x.
    # Defaults reproduce the original mapping; scale_x = 0.0 keeps x frozen.
    # Flip a sign if a direction feels inverted on your setup.
    scale_x: float = 0.0
    scale_y: float = -0.30
    scale_z: float = -0.30
    # Targets are clamped to +-workspace_extent around the reference pose.
    # 0.08 m verified reachable for the patched arm from the shipped home pose.
    workspace_extent: float = 0.08

    # --- re-planning gates ------------------------------------------------
    position_threshold: float = 0.02     # m   (old code: 0.001 with a "2cm" comment)
    orientation_threshold: float = 0.2   # rad
    plan_cooldown: float = 1.0           # s, per arm
    max_missing_frames: int = 5          # hand lost -> reset reference
    # When only one hand has a fresh target, wait this long for the other
    # hand to join so both arms can move in one simultaneous plan.
    merge_window: float = 0.25           # s

    # --- MoveIt request ---------------------------------------------------
    planning_time: float = 5.0
    num_planning_attempts: int = 4
    velocity_scaling: float = 0.5
    acceleration_scaling: float = 0.5
    planner_id: str = ''                 # '' = pipeline default (RRTConnect)
    combined_group: str = 'both_manipulators'  # SRDF group spanning both arms
    position_tolerance: float = 0.02     # goal sphere radius (m)
    # Set use_orientation_constraint False if you run the *unpatched* URDF
    # with position_only_ik (see PATCH_NOTES.md).
    use_orientation_constraint: bool = True
    orientation_tolerance: float = 0.5   # rad per axis; 0.5 samples well,
                                         # the old 1.0 wandered too far
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

    # frame helpers -------------------------------------------------------
    def _suffix(self, robot: str) -> str:
        return '_1' if robot == self.robot1_name else '_2'

    def base_frame(self, robot: str) -> str:
        return f'base_link_robot{self._suffix(robot)}'

    def tip_link(self, robot: str) -> str:
        return f'link6_robot{self._suffix(robot)}'

    def planning_group(self, robot: str) -> str:
        return f'manipulator_{robot}'

    def gripper_action(self, robot: str) -> str:
        return f'/gripper_{robot}_controller/gripper_cmd'
