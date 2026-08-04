"""Hand state -> end-effector target mapping.

Pure numpy, no ROS imports: unit-testable in isolation. The node layer
converts between geometry_msgs types and arrays (see ros_conversions.py).
"""

from typing import Dict, Optional, Tuple

import numpy as np

Pose = Tuple[np.ndarray, np.ndarray]          # (position xyz, quaternion xyzw)

MODE_NORMAL = ''
MODE_EXACT_CURRENT = 'exact_current_pose'
MODE_FIXED_OFFSET = 'fixed_offset'


class TargetMapper:
    """Turns relative hand positions into clamped end-effector targets.

    The first hand sample after a reset only captures the reference pose
    (the arm's pose at that moment) and produces no target; subsequent
    samples produce ``reference + gains * hand_offset`` clamped to a box.
    """

    def __init__(self, gains_xyz: np.ndarray, extent: float,
                 mode: str = MODE_NORMAL):
        self._gains = np.asarray(gains_xyz, dtype=float)
        self._extent = float(extent)
        self._mode = mode
        self._reference: Dict[str, Pose] = {}

    # ------------------------------------------------------------------
    def reset(self, robot: str) -> None:
        """Forget the reference pose (call when the hand disappears)."""
        self._reference.pop(robot, None)

    def has_reference(self, robot: str) -> bool:
        return robot in self._reference

    # ------------------------------------------------------------------
    def compute(self, robot: str, hand_rel_pos: np.ndarray,
                current_pose: Pose) -> Optional[Pose]:
        """Return the target pose, or None while the reference is being set.

        hand_rel_pos: HandState.position as [x, y, z] (already relative to
        the point where the hand first appeared, as published by the
        hand-tracking node).
        current_pose: the arm's current EE pose in its base frame.
        """
        cur_pos, cur_quat = current_pose

        if self._mode == MODE_EXACT_CURRENT:
            return np.array(cur_pos, copy=True), np.array(cur_quat, copy=True)

        if self._mode == MODE_FIXED_OFFSET:
            return cur_pos + np.array([0.0, 0.01, 0.0]), np.array(cur_quat, copy=True)

        # -------- normal hand following --------------------------------
        if robot not in self._reference:
            self._reference[robot] = (np.array(cur_pos, copy=True),
                                      np.array(cur_quat, copy=True))
            return None

        ref_pos, ref_quat = self._reference[robot]
        hand = np.asarray(hand_rel_pos, dtype=float)

        # hand.z -> robot x, hand.x -> robot y, hand.y -> robot z
        offset = np.array([self._gains[0] * hand[2],
                           self._gains[1] * hand[0],
                           self._gains[2] * hand[1]])
        offset = np.clip(offset, -self._extent, self._extent)

        return ref_pos + offset, np.array(ref_quat, copy=True)


def movement_is_significant(prev: Optional[Pose], new: Pose,
                            pos_threshold: float,
                            orient_threshold: float) -> bool:
    """True when *new* differs enough from *prev* to justify a re-plan."""
    if prev is None:
        return True
    d_pos = float(np.linalg.norm(new[0] - prev[0]))
    dot = float(np.clip(abs(np.dot(new[1], prev[1])), -1.0, 1.0))
    d_orient = 2.0 * float(np.arccos(dot))
    return d_pos > pos_threshold or d_orient > orient_threshold
