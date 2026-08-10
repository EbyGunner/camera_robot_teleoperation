"""Per-arm MoveGroup clients, one namespaced move_group per arm.

Each arm now has its own move_group instance, reached at
``/<namespace>/move_action``, so the two arms plan and execute genuinely
in parallel. What that removes, compared with the previous single
move_group + combined ``both_manipulators`` group:

  * no merge window - an arm no longer waits ~0.25 s on the chance that
    the other hand is about to produce a target;
  * no shared failure - a NO_IK_SOLUTION on one arm used to abort the
    combined goal and stop BOTH arms; now it stops only that arm;
  * no alternating goals - each arm re-plans on its own cooldown.

What it costs: the two trajectories are no longer time-synchronised
against each other. Cross-arm collision checking still happens, because
each move_group loads the full dual-arm URDF/SRDF and monitors the global
/joint_states, but it is checked against the other arm's CURRENT state
rather than its future path.

Flow control per arm lives in ArmGate: one goal in flight, freshest
pending target wins, and a watchdog cancels goals that overrun
``goal_timeout``.
"""

from typing import Dict, Optional, Tuple

import numpy as np
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, MoveItErrorCodes
from rclpy.action import ActionClient

from .arm_gate import ArmGate
from .goal_constraints import pose_goal
from .params import ImitationParams

Target = Tuple[np.ndarray, np.ndarray]          # (position, quaternion)

_ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS: 'SUCCESS',
    MoveItErrorCodes.PLANNING_FAILED: 'PLANNING_FAILED',
    MoveItErrorCodes.INVALID_MOTION_PLAN: 'INVALID_MOTION_PLAN',
    MoveItErrorCodes.CONTROL_FAILED: 'CONTROL_FAILED',
    MoveItErrorCodes.TIMED_OUT: 'TIMED_OUT',
    MoveItErrorCodes.START_STATE_IN_COLLISION: 'START_STATE_IN_COLLISION',
    MoveItErrorCodes.GOAL_IN_COLLISION: 'GOAL_IN_COLLISION',
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'GOAL_CONSTRAINTS_VIOLATED',
    MoveItErrorCodes.NO_IK_SOLUTION: 'NO_IK_SOLUTION',
}


class ArmPlanner:
    """MoveGroup client for ONE arm on ONE namespaced move_group."""

    def __init__(self, node, callback_group, params: ImitationParams,
                 robot: str):
        self._node = node
        self._log = node.get_logger()
        self._params = params
        self._robot = robot
        self._action_name = params.move_action(robot)
        self._client = ActionClient(node, MoveGroup, self._action_name,
                                    callback_group=callback_group)
        self._gate = ArmGate()
        self._active_handle = None
        self._available = False

    # ------------------------------------------------------------------
    @property
    def available(self) -> bool:
        return self._available

    def wait_for_server(self, timeout_sec: float) -> bool:
        self._available = self._client.wait_for_server(timeout_sec=timeout_sec)
        if self._available:
            self._log.info(
                f'{self._robot}: move_group ready on {self._action_name}')
        else:
            self._log.warning(
                f'{self._robot}: no move_group on {self._action_name} - '
                f'that arm is disabled. Check that move_group_ns.launch.py '
                f'ran with namespace:={self._params.namespace(self._robot)}.')
        return self._available

    # ------------------------------------------------------------------
    def update_target(self, position: np.ndarray,
                      quaternion: np.ndarray) -> None:
        if not self._available:
            return
        self._gate.update((position, quaternion))
        self.pump()

    def clear_target(self) -> None:
        self._gate.clear()

    # ------------------------------------------------------------------
    def pump(self) -> None:
        """Run the stuck-goal watchdog, then release a queued target."""
        now = self._now()

        age = self._gate.active_age(now)
        if age is not None and age > self._params.goal_timeout:
            self._log.warning(
                f'{self._robot}: goal exceeded '
                f'{self._params.goal_timeout:.0f}s - cancelling')
            handle, self._active_handle = self._active_handle, None
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:  # noqa: BLE001
                    self._log.error(f'{self._robot}: cancel failed: {exc}')
            self._gate.complete()

        target = self._gate.take(now)
        if target is not None:
            self._send(target)

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def _send(self, target: Target) -> None:
        p = self._params
        position, quaternion = target

        request = MotionPlanRequest()
        request.group_name = p.planning_group(self._robot)
        request.planner_id = p.planner_id
        request.allowed_planning_time = p.planning_time
        request.num_planning_attempts = p.num_planning_attempts
        request.max_velocity_scaling_factor = p.velocity_scaling
        request.max_acceleration_scaling_factor = p.acceleration_scaling
        request.start_state.is_diff = True     # plan from the current state
        orient_tol = (p.orientation_tolerance
                      if p.use_orientation_constraint else None)
        request.goal_constraints = [pose_goal(
            p.base_frame(self._robot), p.tip_link(self._robot),
            position, quaternion, p.position_tolerance, orient_tol)]

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        if p.debug:
            self._log.info(
                f'{self._robot} [{request.group_name}] -> '
                f'({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})')
        try:
            future = self._client.send_goal_async(goal)
            future.add_done_callback(self._on_goal_response)
        except Exception as exc:  # noqa: BLE001
            self._log.error(f'{self._robot}: send_goal failed: {exc}')
            self._finish()

    # ------------------------------------------------------------------
    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._log.warning(f'{self._robot}: planning goal rejected')
                self._finish()
                return
            self._active_handle = handle
            handle.get_result_async().add_done_callback(self._on_result)
        except Exception as exc:  # noqa: BLE001
            self._log.error(f'{self._robot}: goal response error: {exc}')
            self._finish()

    def _on_result(self, future) -> None:
        try:
            code = future.result().result.error_code.val
            name = _ERROR_NAMES.get(code, str(code))
            if code == MoveItErrorCodes.SUCCESS:
                if self._params.debug:
                    self._log.info(f'{self._robot}: plan+execute succeeded')
            else:
                self._log.warning(
                    f'{self._robot}: planning/execution failed ({name})')
        except Exception as exc:  # noqa: BLE001
            self._log.error(f'{self._robot}: result handling error: {exc}')
        finally:
            self._finish()

    def _finish(self) -> None:
        self._active_handle = None
        self._gate.complete()
        self.pump()                      # send anything queued meanwhile

    # ------------------------------------------------------------------
    def cancel(self) -> None:
        handle, self._active_handle = self._active_handle, None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001 - best-effort shutdown
                pass


class PlannerFleet:
    """One ArmPlanner per arm, with the API the node used to call.

    Deliberately a drop-in for the old MoveGroupPlanner so imitation_node
    only had to change its import: the arm-independence lives entirely
    below this line.
    """

    def __init__(self, node, callback_group, params: ImitationParams):
        self._log = node.get_logger()
        self._planners: Dict[str, ArmPlanner] = {
            robot: ArmPlanner(node, callback_group, params, robot)
            for robot in params.robots
        }

    # ------------------------------------------------------------------
    def wait_for_server(self, timeout_sec: float) -> bool:
        """Wait for every arm's move_group. True if at least one is up.

        The arms are independent now, so one missing move_group disables
        one arm instead of the whole node. The first call absorbs the
        timeout; the rest resolve immediately if both were launched
        together.
        """
        ready = [planner.wait_for_server(timeout_sec)
                 for planner in self._planners.values()]
        if not any(ready):
            self._log.warning('No move_group instances found - arm motion '
                              'disabled (grippers still work)')
        return any(ready)

    # ------------------------------------------------------------------
    def update_target(self, robot: str, position: np.ndarray,
                      quaternion: np.ndarray) -> None:
        planner = self._planners.get(robot)
        if planner is not None:
            planner.update_target(position, quaternion)

    def clear_target(self, robot: str) -> None:
        planner = self._planners.get(robot)
        if planner is not None:
            planner.clear_target()

    def pump(self) -> None:
        for planner in self._planners.values():
            planner.pump()

    def cancel_all(self) -> None:
        for planner in self._planners.values():
            planner.cancel()

    # ------------------------------------------------------------------
    def get(self, robot: str) -> Optional[ArmPlanner]:
        return self._planners.get(robot)