#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from robot_interfaces.msg import HandState
import cv2
import os
import sys

# Add external library path
SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src'))
EXT_LIB_PATH = os.path.join(SRC_DIR, 'hand_tracking', 'external_libraries')
sys.path.insert(0, EXT_LIB_PATH)

import mediapipe as mp

class HandDetector:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.8)
    
    def detect_hands(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        
        detected_hands = {"left": None, "right": None}
        
        if results.multi_hand_landmarks:
            for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                handedness = results.multi_handedness[i].classification[0].label.lower()
                
                # Get wrist position
                wrist = hand_landmarks.landmark[0]
                position = Point(x=wrist.x, y=wrist.y, z=0.0)
                
                # Detect if hand is closed
                is_closed = self._is_hand_closed(hand_landmarks)
                
                detected_hands[handedness] = {
                    "position": position,
                    "is_closed": is_closed
                }
        
        return detected_hands
    
    def _is_hand_closed(self, landmarks):
        # Check finger states (thumb to pinky)
        finger_states = []
        
        # Thumb (different calculation)
        thumb_tip = landmarks.landmark[4]
        thumb_mcp = landmarks.landmark[2]
        thumb_state = 1 if thumb_tip.x < thumb_mcp.x else 0
        
        # Other fingers
        for tip, pip in [(8,6), (12,10), (16,14), (20,18)]:
            tip_point = landmarks.landmark[tip]
            pip_point = landmarks.landmark[pip]
            finger_state = 1 if tip_point.y < pip_point.y else 0
            finger_states.append(finger_state)
        
        # A fist has most fingers bent
        return sum([thumb_state] + finger_states) < 2

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')

        # Create publishers for both hands
        self.hand_publishers = {
            'left': self.create_publisher(HandState, '/left_hand_state', 10),
            'right': self.create_publisher(HandState, '/right_hand_state', 10)
        }

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.detector = HandDetector()
        self.cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video device")
        else:
            self.get_logger().info('Hand Tracking Node started')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return
        
        frame = cv2.flip(frame, 1)  # Mirror for natural interaction
        detected_hands = self.detector.detect_hands(frame)

        for hand_type in ['left', 'right']:
            hand_data = detected_hands[hand_type]
            msg = HandState()
            msg.hand_type = hand_type
            msg.detected = hand_data is not None

            if hand_data:
                msg.position = hand_data["position"]
                msg.is_closed = hand_data["is_closed"]
                self.get_logger().info(
                    f"Published {hand_type} hand at "
                    f"({msg.position.x:.2f}, {msg.position.y:.2f}) "
                    f"{'CLOSED' if msg.is_closed else 'OPEN'}"
                )
            else:
                self.get_logger().info(f"{hand_type.capitalize()} hand not detected")
            
            self.hand_publishers[hand_type].publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
