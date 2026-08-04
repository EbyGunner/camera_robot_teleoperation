"""Builds MoveIt goal Constraints for end-effector pose targets.

All constraints for one request go into a SINGLE Constraints message:
entries of MotionPlanRequest.goal_constraints are ALTERNATIVES (OR), so a
simultaneous dual-arm goal must AND both arms' constraints inside one
element - that is what multi_pose_goal produces.
"""

from typing import Iterable, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive

from .ros_conversions import np_to_pose

# (base_frame, link_name, position xyz, quaternion xyzw)
PoseTarget = Tuple[str, str, np.ndarray, np.ndarray]


def multi_pose_goal(targets: Iterable[PoseTarget],
                    position_tolerance: float,
                    orientation_tolerance: Optional[float]) -> Constraints:
    """One Constraints message covering every (frame, link, pose) target.

    Pass orientation_tolerance=None to omit orientation constraints
    entirely (required when the KDL solver runs with position_only_ik:
    the solved orientation is then arbitrary and an orientation constraint
    would reject nearly every sampled goal state).
    """
    constraints = Constraints()
    for base_frame, link_name, position, quaternion in targets:
        _append_pose(constraints, base_frame, link_name, position,
                     quaternion, position_tolerance, orientation_tolerance)
    return constraints


def pose_goal(base_frame: str, link_name: str,
              position: np.ndarray, quaternion: np.ndarray,
              position_tolerance: float,
              orientation_tolerance: Optional[float]) -> Constraints:
    """Single-link convenience wrapper around multi_pose_goal."""
    return multi_pose_goal([(base_frame, link_name, position, quaternion)],
                           position_tolerance, orientation_tolerance)


def _append_pose(constraints: Constraints, base_frame: str, link_name: str,
                 position: np.ndarray, quaternion: np.ndarray,
                 position_tolerance: float,
                 orientation_tolerance: Optional[float]) -> None:

    pc = PositionConstraint()
    pc.header.frame_id = base_frame
    pc.link_name = link_name
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [float(position_tolerance)]
    pc.constraint_region.primitives.append(sphere)
    region_pose = Pose()
    region_pose.position.x = float(position[0])
    region_pose.position.y = float(position[1])
    region_pose.position.z = float(position[2])
    region_pose.orientation.w = 1.0
    pc.constraint_region.primitive_poses.append(region_pose)
    pc.weight = 1.0
    constraints.position_constraints.append(pc)

    if orientation_tolerance is not None:
        oc = OrientationConstraint()
        oc.header.frame_id = base_frame
        oc.link_name = link_name
        oc.orientation = np_to_pose(position, quaternion).orientation
        oc.absolute_x_axis_tolerance = float(orientation_tolerance)
        oc.absolute_y_axis_tolerance = float(orientation_tolerance)
        oc.absolute_z_axis_tolerance = float(orientation_tolerance)
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
