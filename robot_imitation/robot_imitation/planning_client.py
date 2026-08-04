"""Batched MoveGroup client: one goal at a time, covering one or both arms.

move_group executes a single motion goal at a time, so simultaneous
dual-arm motion is only possible by planning both arms as ONE group in
ONE request. This client keeps the freshest target per arm in a
MergeDispatcher; whenever a batch is released it sends either a
single-arm goal (that arm's group) or a combined goal (the
``both_manipulators`` SRDF group, constraints for both tip links), whose
trajectory move_group splits across both arm controllers and executes
concurrently. A watchdog cancels goals stuck past ``goal_timeout``.
"""

from typing import Dict, Optional, Tuple

import numpy as np
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, MoveItErrorCodes
from rclpy.action import ActionClient

from .goal_constraints import multi_pose_goal
from .merge_dispatch import MergeDispatcher
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


class MoveGroupPlanner:
    def __init__(self, node, callback_group, params: ImitationParams):
        self._node = node
        self._log = node.get_logger()
        self._params = params
        self._client = ActionClient(node, MoveGroup, '/move_action',
                                    callback_group=callback_group)
        self._dispatcher = MergeDispatcher(
            [params.robot1_name, params.robot2_name], params.merge_window)
        self._active_handle = None
        self._available = False

    # ------------------------------------------------------------------
    def wait_for_server(self, timeout_sec: float) -> bool:
        self._available = self._client.wait_for_server(timeout_sec=timeout_sec)
        if not self._available:
            self._log.warning('MoveGroup action server not available - '
                              'arm motion disabled')
        return self._available

    # ------------------------------------------------------------------
    def update_target(self, robot: str, position: np.ndarray,
                      quaternion: np.ndarray) -> None:
        """Record this arm's freshest target and dispatch if possible."""
        if not self._available:
            return
        self._dispatcher.update(robot, (position, quaternion), self._now())
        self.pump()

    def clear_target(self, robot: str) -> None:
        self._dispatcher.clear(robot)

    # ------------------------------------------------------------------
    def pump(self) -> None:
        """Called on updates, on results, and by a periodic timer:
        runs the stuck-goal watchdog and releases the next batch."""
        now = self._now()

        age = self._dispatcher.active_age(now)
        if age is not None and age > self._params.goal_timeout:
            self._log.warning(
                f'Goal for {"+".join(self._dispatcher.busy_keys)} exceeded '
                f'{self._params.goal_timeout:.0f}s - cancelling')
            handle, self._active_handle = self._active_handle, None
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:  # noqa: BLE001
                    self._log.error(f'Cancel failed: {exc}')
            self._dispatcher.complete(now)

        batch = self._dispatcher.take_batch(now)
        if batch is not None:
            self._send(batch)

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def _send(self, batch: Dict[str, Target]) -> None:
        p = self._params
        robots = list(batch.keys())

        request = MotionPlanRequest()
        request.group_name = (p.planning_group(robots[0]) if len(robots) == 1
                              else p.combined_group)
        request.planner_id = p.planner_id
        request.allowed_planning_time = p.planning_time
        request.num_planning_attempts = p.num_planning_attempts
        request.max_velocity_scaling_factor = p.velocity_scaling
        request.max_acceleration_scaling_factor = p.acceleration_scaling
        request.start_state.is_diff = True     # plan from the current state
        orient_tol = (p.orientation_tolerance
                      if p.use_orientation_constraint else None)
        request.goal_constraints = [multi_pose_goal(
            [(p.base_frame(r), p.tip_link(r), *batch[r]) for r in robots],
            p.position_tolerance, orient_tol)]

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        if p.debug:
            desc = ', '.join(
                f'{r}->({batch[r][0][0]:.3f}, {batch[r][0][1]:.3f}, '
                f'{batch[r][0][2]:.3f})' for r in robots)
            self._log.info(f'Planning [{request.group_name}]: {desc}')
        try:
            future = self._client.send_goal_async(goal)
            future.add_done_callback(self._on_goal_response)
        except Exception as exc:  # noqa: BLE001
            self._log.error(f'send_goal failed for {robots}: {exc}')
            self._finish()

    # ------------------------------------------------------------------
    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._log.warning('Planning goal rejected')
                self._finish()
                return
            self._active_handle = handle
            handle.get_result_async().add_done_callback(self._on_result)
        except Exception as exc:  # noqa: BLE001
            self._log.error(f'Goal response error: {exc}')
            self._finish()

    def _on_result(self, future) -> None:
        arms = '+'.join(self._dispatcher.busy_keys)
        try:
            code = future.result().result.error_code.val
            name = _ERROR_NAMES.get(code, str(code))
            if code == MoveItErrorCodes.SUCCESS:
                if self._params.debug:
                    self._log.info(f'{arms}: plan+execute succeeded')
            else:
                self._log.warning(f'{arms}: planning/execution failed ({name})')
        except Exception as exc:  # noqa: BLE001
            self._log.error(f'Result handling error: {exc}')
        finally:
            self._finish()

    def _finish(self) -> None:
        self._active_handle = None
        self._dispatcher.complete(self._now())
        self.pump()                      # dispatch anything queued meanwhile

    # ------------------------------------------------------------------
    def cancel_all(self) -> None:
        handle, self._active_handle = self._active_handle, None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001 - best-effort shutdown
                pass
