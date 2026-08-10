"""Current end-effector pose lookup via TF.

Non-blocking: lookups either succeed against the buffer immediately or
return None (the 10 Hz hand stream retries naturally). 
"""

from typing import Optional, Tuple

import numpy as np
import rclpy
from tf2_ros import Buffer, TransformException, TransformListener

from .ros_conversions import transform_to_np


class EndEffectorTracker:
    def __init__(self, node):
        self._log = node.get_logger()
        self._buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self._listener = TransformListener(self._buffer, node)

    def get_pose(self, base_frame: str, tip_link: str
                 ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """(position, quaternion) of *tip_link* in *base_frame*, or None."""
        try:
            tf = self._buffer.lookup_transform(
                base_frame, tip_link, rclpy.time.Time())
            return transform_to_np(tf)
        except TransformException as exc:
            self._log.warning(
                f'TF {base_frame} <- {tip_link} unavailable: {exc}',
                throttle_duration_sec=5.0)
            return None
