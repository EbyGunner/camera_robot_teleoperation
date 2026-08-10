"""Small helpers converting between geometry_msgs types and numpy arrays.

Keeping conversions here lets pose_mapping.py and merge_dispatch.py stay free of
ROS imports (and therefore trivially unit-testable).
"""

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion


def point_to_np(p: Point) -> np.ndarray:
    return np.array([p.x, p.y, p.z], dtype=float)


def quat_to_np(q: Quaternion) -> np.ndarray:
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


def np_to_pose(position: np.ndarray, quaternion: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (
        float(position[0]), float(position[1]), float(position[2]))
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = (
        float(quaternion[0]), float(quaternion[1]),
        float(quaternion[2]), float(quaternion[3]))
    return pose


def transform_to_np(tf) -> tuple:
    """geometry_msgs/TransformStamped -> (pos xyz, quat xyzw)."""
    t, r = tf.transform.translation, tf.transform.rotation
    return (np.array([t.x, t.y, t.z], dtype=float),
            np.array([r.x, r.y, r.z, r.w], dtype=float))