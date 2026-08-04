"""Per-hand reference tracking: absolute detections -> relative motion.

When a hand first appears, its position / orientation / apparent size are
stored as the reference; later frames report scaled offsets from that
reference. Pure numpy - the ROS node feeds it observations and packs the
results into HandState messages.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .hand_geometry import IDENTITY_QUAT, relative_quat


@dataclass
class _Reference:
    position_xy: np.ndarray      # normalised image coords at first sight
    orientation: np.ndarray      # quaternion [x,y,z,w]
    size: float                  # apparent hand size at first sight


class RelativeTracker:
    def __init__(self, scale_xy: float = 0.5, scale_z: float = 0.1):
        self._scale_xy = float(scale_xy)
        self._scale_z = float(scale_z)
        self._refs: Dict[str, _Reference] = {}

    # ------------------------------------------------------------------
    def reset(self, hand: str) -> None:
        self._refs.pop(hand, None)

    def has_reference(self, hand: str) -> bool:
        return hand in self._refs

    # ------------------------------------------------------------------
    def update(self, hand: str, position_xy: np.ndarray,
               orientation: np.ndarray, size: float
               ) -> Tuple[np.ndarray, np.ndarray]:
        """Feed one observation; returns (rel_position_xyz, rel_quat).

        The first observation after a reset establishes the reference and
        returns zeros/identity (matching the original behaviour).
        """
        ref: Optional[_Reference] = self._refs.get(hand)
        if ref is None:
            self._refs[hand] = _Reference(
                np.array(position_xy[:2], dtype=float),
                np.array(orientation, dtype=float),
                float(size))
            return np.zeros(3), IDENTITY_QUAT.copy()

        rel = np.zeros(3)
        rel[:2] = (np.asarray(position_xy[:2], dtype=float)
                   - ref.position_xy) * self._scale_xy
        if ref.size > 1e-9:
            rel[2] = (size - ref.size) / ref.size * self._scale_z
        return rel, relative_quat(ref.orientation, orientation)
