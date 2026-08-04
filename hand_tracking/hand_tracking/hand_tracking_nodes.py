#!/usr/bin/env python3
"""Webcam hand-tracking node (thin wiring layer).

Logic lives in the sibling modules:
    mediapipe_loader.py  robust mediapipe import
    hand_detector.py     mediapipe wrapper -> HandObservation
    hand_geometry.py     orientation / size / closed heuristics
    relative_tracker.py  first-sight reference + relative offsets

Publishes (unchanged topics / message semantics):
    /left_hand_state, /right_hand_state   robot_interfaces/HandState
    /hand_tracking_video                  sensor_msgs/Image (annotated)
"""

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from robot_interfaces.msg import HandState

from .hand_detector import HandDetector
from .relative_tracker import RelativeTracker

_HAND_TYPE = {'left': HandState.HAND_TYPE_LEFT,
              'right': HandState.HAND_TYPE_RIGHT}


class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')

        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('publish_video', True)
        self.declare_parameter('mirror_image', True)
        device = self.get_parameter('video_device').value
        rate = float(self.get_parameter('rate_hz').value)
        self._publish_video = bool(self.get_parameter('publish_video').value)
        self._mirror = bool(self.get_parameter('mirror_image').value)

        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self.get_logger().error(f'Failed to open video device {device}')
            raise RuntimeError(f'Could not open video device {device}')

        self._detector = HandDetector()
        self._tracker = RelativeTracker(scale_xy=0.5, scale_z=0.1)
        self._bridge = CvBridge()
        self._was_present = {'left': False, 'right': False}

        # Named to avoid shadowing rclpy.Node's internal '_publishers' list
        # (Jazzy's create_publisher appends to it; a dict here broke that).
        self._hand_state_pubs = {
            'left': self.create_publisher(HandState, '/left_hand_state', 10),
            'right': self.create_publisher(HandState, '/right_hand_state', 10),
        }
        self._video_pub = self.create_publisher(
            Image, '/hand_tracking_video', 10)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Hand tracking started on {device} at {rate:.0f} Hz')

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame',
                                    throttle_duration_sec=5.0)
            return
        if self._mirror:
            frame = cv2.flip(frame, 1)

        observations, annotated = self._detector.detect(frame)

        for hand in ('left', 'right'):
            self._hand_state_pubs[hand].publish(
                self._build_msg(hand, observations[hand]))

        if self._publish_video:
            try:
                img = self._bridge.cv2_to_imgmsg(annotated, 'bgr8')
                img.header.stamp = self.get_clock().now().to_msg()
                self._video_pub.publish(img)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Video publish failed: {exc}')

    # ------------------------------------------------------------------
    def _build_msg(self, hand: str, obs) -> HandState:
        msg = HandState()
        msg.hand_type = _HAND_TYPE[hand]
        msg.detected = obs is not None

        if obs is None:
            self._tracker.reset(hand)
            msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.is_closed = False
        else:
            rel_pos, rel_quat = self._tracker.update(
                hand, obs.position_xy, obs.orientation, obs.size)
            msg.position.x, msg.position.y, msg.position.z = (
                float(rel_pos[0]), float(rel_pos[1]), float(rel_pos[2]))
            msg.orientation = Quaternion(
                x=float(rel_quat[0]), y=float(rel_quat[1]),
                z=float(rel_quat[2]), w=float(rel_quat[3]))
            msg.is_closed = bool(obs.is_closed)

        if msg.detected != self._was_present[hand]:
            state = 'appeared' if msg.detected else 'disappeared'
            self.get_logger().info(f'{hand.capitalize()} hand {state}')
            self._was_present[hand] = msg.detected
        return msg

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._cap.release()
        self._detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node: Optional[HandTrackingNode] = None
    try:
        node = HandTrackingNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
