"""Geometry computed from a 21x3 array of mediapipe hand landmarks.

Pure numpy/scipy - no ROS or mediapipe imports - so every function here is
unit-testable with synthetic landmark arrays.

Landmark indices used (mediapipe convention):
    0 wrist, 2 thumb MCP, 4 thumb tip, 5 index MCP, 9 middle MCP,
    12 middle tip, 17 pinky MCP, and (tip, pip) pairs for the fingers.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])

_FINGER_TIP_PIP = ((8, 6), (12, 10), (16, 14), (20, 18))


def orientation_quat(lm: np.ndarray) -> np.ndarray:
    """Palm orientation as quaternion [x, y, z, w].

    X axis: wrist -> middle MCP. Y axis: palm normal. Z: their cross
    product (re-orthogonalised). Falls back to identity on degenerate
    input.
    """
    wrist, middle_mcp = lm[0], lm[9]
    index_mcp, pinky_mcp = lm[5], lm[17]

    x_axis = middle_mcp - wrist
    y_axis = np.cross(index_mcp - wrist, pinky_mcp - wrist)
    if np.linalg.norm(x_axis) < 1e-6 or np.linalg.norm(y_axis) < 1e-6:
        return IDENTITY_QUAT.copy()

    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)

    try:
        return R.from_matrix(np.column_stack([x_axis, y_axis, z_axis])).as_quat()
    except ValueError:
        return IDENTITY_QUAT.copy()


def hand_size(lm: np.ndarray) -> float:
    """Wrist to middle-fingertip distance in image coordinates (used as a
    proxy for hand-to-camera distance)."""
    return float(np.linalg.norm(lm[12, :2] - lm[0, :2]))


def is_closed(lm: np.ndarray) -> bool:
    """Hand considered closed when fewer than two digits are extended.

    Finger extension: tip above PIP in the image (image y grows
    downward). Thumb: the original code compared raw x coordinates,
    which inverts for one of the two hands in a mirrored image. Instead
    we fold-test the thumb along the hand's own index->pinky direction,
    which is handedness- and mirroring-agnostic: a folded thumb tip moves
    toward the pinky side of the palm.
    """
    extended = [1 if lm[tip, 1] < lm[pip, 1] else 0
                for tip, pip in _FINGER_TIP_PIP]

    pinky_dir = lm[17, 0] - lm[5, 0]            # index MCP -> pinky MCP, x
    thumb_travel = lm[4, 0] - lm[2, 0]          # thumb MCP -> tip, x
    thumb_folded = thumb_travel * pinky_dir > 0
    extended.append(0 if thumb_folded else 1)

    return sum(extended) < 2


def relative_quat(initial: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Rotation taking *initial* to *current* (both [x,y,z,w])."""
    rel = R.from_quat(current) * R.from_quat(initial).inv()
    return rel.as_quat()
