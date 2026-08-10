"""Non-blocking gripper control for both arms.
"""

from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

from .params import ImitationParams


class GripperFleet:
    def __init__(self, node, callback_group, params: ImitationParams):
        self._log = node.get_logger()
        self._params = params
        self._clients = {
            robot: ActionClient(node, GripperCommand,
                                params.gripper_action(robot),
                                callback_group=callback_group)
            for robot in (params.robot1_name, params.robot2_name)
        }
        self._last_sent = {}

    # ------------------------------------------------------------------
    def wait_for_servers(self, timeout_sec: float) -> None:
        """Best-effort readiness check at startup (non-fatal)."""
        for robot, client in self._clients.items():
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self._log.warning(
                    f'Gripper server for {robot} not up yet - will retry '
                    f'when commands are sent')

    # ------------------------------------------------------------------
    def command(self, robot: str, closed: bool) -> None:
        target = (self._params.gripper_closed_position if closed
                  else self._params.gripper_open_position)
        if self._last_sent.get(robot) == target:
            return

        client = self._clients[robot]
        if not client.server_is_ready():
            self._log.warning(f'Gripper server not ready for {robot}',
                              throttle_duration_sec=5.0)
            return  # do not record: retry on the next state change

        self._last_sent[robot] = target
        goal = GripperCommand.Goal()
        goal.command.position = float(target)
        goal.command.max_effort = float(self._params.gripper_effort)
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f, r=robot: self._on_response(f, r))
        if self._params.debug:
            self._log.info(
                f"Gripper {robot}: {'CLOSE' if closed else 'OPEN'}")

    # ------------------------------------------------------------------
    def _on_response(self, future, robot: str) -> None:
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._log.warning(f'Gripper goal rejected for {robot}')
                self._last_sent.pop(robot, None)   # allow retry
        except Exception as exc:  # noqa: BLE001
            self._log.error(f'Gripper response error for {robot}: {exc}')
            self._last_sent.pop(robot, None)
