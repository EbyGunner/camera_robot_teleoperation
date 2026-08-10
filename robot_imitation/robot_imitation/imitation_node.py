#!/usr/bin/env python3
"""Hand-to-robot teleoperation node (thin wiring layer).

All real logic lives in the sibling modules:

    params.py           parameter schema + frame/action helpers
    pose_mapping.py     hand offsets -> clamped EE targets (pure numpy)
    arm_gate.py         per-arm "one goal in flight, newest wins" gate
    goal_constraints.py MoveIt Constraints construction
    planning_client.py  one MoveGroup client per namespaced move_group
    gripper_client.py   gripper actions
    ee_tracker.py       current EE pose from TF
"""

import threading
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_interfaces.msg import HandState

from .ee_tracker import EndEffectorTracker
from .gripper_client import GripperFleet
from .params import ImitationParams
from .planning_client import PlannerFleet
from .pose_mapping import (MODE_NORMAL, TargetMapper,
                           movement_is_significant)
from .ros_conversions import point_to_np


class HandToRobotController(Node):
    def __init__(self):
        super().__init__('hand_to_robot_controller')
        self.get_logger().info('Hand to Robot Controller starting')

        self.params = ImitationParams.from_node(self)
        p = self.params

        if p.diagnostic_mode:
            self.get_logger().warning(
                f"diagnostic_mode='{p.diagnostic_mode}': hand input is "
                f"IGNORED. Set diagnostic_mode:='' for teleoperation.")

        self.get_logger().info(
            f'Arms: {p.robot1_name} -> {p.move_action(p.robot1_name)}, '
            f'{p.robot2_name} -> {p.move_action(p.robot2_name)}')

        self._cb_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()

        # components -----------------------------------------------------
        self.mapper = TargetMapper(
            np.array([p.scale_x, p.scale_y, p.scale_z]),
            p.workspace_extent, p.diagnostic_mode)
        self.tracker = EndEffectorTracker(self)
        self.planner = PlannerFleet(self, self._cb_group, p)
        self.grippers = GripperFleet(self, self._cb_group, p)

        self.planner.wait_for_server(timeout_sec=15.0)
        self.grippers.wait_for_servers(timeout_sec=2.0)

        # per-robot bookkeeping -----------------------------------------
        self._missing_frames: Dict[str, int] = {r: 0 for r in p.robots}
        self._last_sent_target = {r: None for r in p.robots}
        self._last_plan_time: Dict[str, float] = {r: 0.0 for r in p.robots}

        # subscriptions & timers ----------------------------------------
        self.create_subscription(
            HandState, '/left_hand_state',
            lambda m: self._on_hand(p.robot1_name, m), 10,
            callback_group=self._cb_group)
        self.create_subscription(
            HandState, '/right_hand_state',
            lambda m: self._on_hand(p.robot2_name, m), 10,
            callback_group=self._cb_group)
        # drives per-arm dispatch and the stuck-goal watchdogs
        self.create_timer(0.05, self.planner.pump,
                          callback_group=self._cb_group)

        self.get_logger().info(
            'Ready - listening on /left_hand_state and /right_hand_state')

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_hand(self, robot: str, msg: HandState) -> None:
        try:
            if not msg.detected:
                self._on_hand_lost(robot)
                return
            self._missing_frames[robot] = 0

            if self.params.debug:
                self.get_logger().info(
                    f'{robot} hand: ({msg.position.x:+.3f}, '
                    f'{msg.position.y:+.3f}, {msg.position.z:+.3f}) '
                    f"{'CLOSED' if msg.is_closed else 'OPEN'}",
                    throttle_duration_sec=1.0)

            self.grippers.command(robot, msg.is_closed)
            self._maybe_plan(robot, msg)
        except Exception as exc:  # noqa: BLE001 - keep the stream alive
            self.get_logger().error(f'{robot} hand handling failed: {exc}')

    def _on_hand_lost(self, robot: str) -> None:
        self._missing_frames[robot] += 1
        if self._missing_frames[robot] == self.params.max_missing_frames:
            self.get_logger().info(f'{robot}: hand lost - reference reset')
            with self._state_lock:
                self.mapper.reset(robot)
                self._last_sent_target[robot] = None
            self.planner.clear_target(robot)

    # ------------------------------------------------------------------
    def _maybe_plan(self, robot: str, msg: HandState) -> None:
        p = self.params
        current = self.tracker.get_pose(p.base_frame(robot),
                                        p.tip_link(robot))
        if current is None:
            return

        with self._state_lock:
            target = self.mapper.compute(
                robot, point_to_np(msg.position), current)
            if target is None:      # reference pose was just captured
                self.get_logger().info(f'{robot}: reference pose set')
                return

            if p.diagnostic_mode == MODE_NORMAL and not movement_is_significant(
                    self._last_sent_target[robot], target,
                    p.position_threshold, p.orientation_threshold):
                return

            now = self._now()
            if now - self._last_plan_time[robot] < p.plan_cooldown:
                return
            self._last_plan_time[robot] = now
            self._last_sent_target[robot] = target

        self.planner.update_target(robot, *target)

    # ------------------------------------------------------------------
    def destroy_node(self):
        # On Ctrl-C rclpy's signal handler has already invalidated the
        # context: logging fails and cancel_goal_async() has no transport
        # left. Only clean up when the context is still alive, i.e. on a
        # programmatic shutdown. On Ctrl-C move_group gets its own SIGINT
        # and stops the arms anyway.
        if rclpy.ok():
            self.get_logger().info('Shutting down - cancelling active goals')
            self.planner.cancel_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node: Optional[HandToRobotController] = None
    try:
        node = HandToRobotController()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()