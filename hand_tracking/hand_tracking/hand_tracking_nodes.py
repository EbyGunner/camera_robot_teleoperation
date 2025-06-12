#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from robot_interfaces.msg import HandState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import signal
from rclpy.executors import MultiThreadedExecutor

# Add external library path
SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src'))
EXT_LIB_PATH = os.path.join(SRC_DIR, 'camera_robot_teleoperation', 'hand_tracking', 'external_libraries')
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
        self.mp_drawing = mp.solutions.drawing_utils
    
    def detect_hands(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        detected_hands = {"left": None, "right": None}
        annotated_image = frame.copy()

        if results.multi_hand_landmarks:
            for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                handedness = results.multi_handedness[i].classification[0].label.lower()
                if detected_hands[handedness] is not None:
                    continue  # Ignore additional hands of same type

                wrist = hand_landmarks.landmark[0]
                position = Point(x=wrist.x, y=wrist.y, z=0.0)
                is_closed = self._is_hand_closed(hand_landmarks)

                landmark_color = (0, 0, 255) if is_closed else (0, 255, 0)
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                    mp.solutions.drawing_styles.get_default_hand_connections_style())

                text_x = int(wrist.x * frame.shape[1])
                text_y = int(wrist.y * frame.shape[0]) - 20
                cv2.putText(annotated_image,
                            f"{handedness} {'CLOSED' if is_closed else 'OPEN'}",
                            (text_x, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, landmark_color, 2)

                detected_hands[handedness] = {
                    "position": position,
                    "is_closed": is_closed
                }

        return detected_hands, annotated_image, results.multi_hand_landmarks, results.multi_handedness    
    
    def compute_hand_size(self, landmarks):
        # Use wrist (0) and middle fingertip (12) as a distance metric
        wrist = landmarks.landmark[0]
        middle_fingertip = landmarks.landmark[12]
        dx = middle_fingertip.x - wrist.x
        dy = middle_fingertip.y - wrist.y
        return (dx**2 + dy**2) ** 0.5
    
    def _is_hand_closed(self, landmarks):
        finger_states = []
        
        # Thumb
        thumb_tip = landmarks.landmark[4]
        thumb_mcp = landmarks.landmark[2]
        thumb_state = 1 if thumb_tip.x < thumb_mcp.x else 0
        
        # Other fingers
        for tip, pip in [(8,6), (12,10), (16,14), (20,18)]:
            tip_point = landmarks.landmark[tip]
            pip_point = landmarks.landmark[pip]
            finger_state = 1 if tip_point.y < pip_point.y else 0
            finger_states.append(finger_state)
        
        return sum([thumb_state] + finger_states) < 2

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')

        self.running = True

        self.hand_publishers = {
            'left': self.create_publisher(HandState, '/left_hand_state', 10),
            'right': self.create_publisher(HandState, '/right_hand_state', 10)
        }
        self.video_pub = self.create_publisher(Image, '/hand_tracking_video', 10)
        self.bridge = CvBridge()

        self.detector = HandDetector()
        self.cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

        self.initial_positions = {"left": None, "right": None}
        self.initial_hand_sizes = {"left": None, "right": None}
        self.latest_detected_hands = {"left": None, "right": None}


        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video device")
            raise RuntimeError("Could not open video device")

        self.get_logger().info('Hand Tracking Node started with video publishing')

        # Timer for hand state messages (1 Hz)
        self.hand_state_timer = self.create_timer(1.0, self.hand_state_timer_callback)
        # Timer for video feed (10 Hz)
        self.video_timer = self.create_timer(0.1, self.video_timer_callback)

        # Track previously detected states
        self.prev_hand_presence = {"left": False, "right": False}


    def video_timer_callback(self):
        """Process video and update latest hand detections at 10Hz"""
        if not self.running:
            return

        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        frame = cv2.flip(frame, 1)
        detected_hands, annotated_frame, landmarks_list, handedness_list = self.detector.detect_hands(frame)
        
        # Store the latest detection results
        self.latest_detected_hands = detected_hands
        self.latest_landmarks_list = landmarks_list or []
        self.latest_handedness_list = handedness_list or []

        # Publish annotated video
        try:
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            self.video_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Failed to convert and publish image: {str(e)}")



    def hand_state_timer_callback(self):
        """Publish hand states at 1Hz using the latest detection results"""
        if not self.running:
            return

        detected_hands = self.latest_detected_hands
        landmarks_list = self.latest_landmarks_list
        handedness_list = self.latest_handedness_list

        for i, hand_type in enumerate(['left', 'right']):
            msg = HandState()
            msg.hand_type = hand_type
            msg.detected = detected_hands[hand_type] is not None

            if msg.detected:
                hand_info = detected_hands[hand_type]
                raw_position = hand_info["position"]
                msg.is_closed = hand_info["is_closed"]

                # Set initial relative origin
                if self.initial_hand_sizes[hand_type] is None:
                    # First appearance
                    for j, classification in enumerate(handedness_list):
                        if classification.classification[0].label.lower() == hand_type:
                            hand_landmarks = landmarks_list[j]
                            hand_size = self.detector.compute_hand_size(hand_landmarks)
                            self.initial_hand_sizes[hand_type] = hand_size
                            self.initial_positions[hand_type] = (raw_position.x, raw_position.y)
                            msg.position.x = 0.0
                            msg.position.y = 0.0
                            msg.position.z = 0.0
                            break
                else:
                    scale_x = 0.5
                    scale_y = 0.5
                    scale_z = 0.1

                    # Relative motion
                    origin_x, origin_y = self.initial_positions[hand_type]
                    msg.position.x = (raw_position.x - origin_x) * scale_x
                    msg.position.y = (raw_position.y - origin_y) * scale_y

                    for j, classification in enumerate(handedness_list):
                        if classification.classification[0].label.lower() == hand_type:
                            hand_landmarks = landmarks_list[j]
                            hand_size = self.detector.compute_hand_size(hand_landmarks)
                            msg.position.z = (hand_size / self.initial_hand_sizes[hand_type]) * scale_z
                            break
            else:
                # Hand not detected
                msg.position.x = 0.0
                msg.position.y = 0.0
                msg.position.z = 0.0
                self.initial_hand_sizes[hand_type] = None
                self.initial_positions[hand_type] = None

            # Log only on appearance/disappearance
            if msg.detected != self.prev_hand_presence[hand_type]:
                if msg.detected:
                    self.get_logger().info(
                        f"{hand_type.capitalize()} hand appeared at "
                        f"({msg.position.x:.2f}, {msg.position.y:.2f}, z={msg.position.z:.2f}) - "
                        f"{'CLOSED' if msg.is_closed else 'OPEN'}"
                        )
                else:
                    self.get_logger().info(f"{hand_type.capitalize()} hand disappeared")
                self.prev_hand_presence[hand_type] = msg.detected

            self.hand_publishers[hand_type].publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HandTrackingNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        def sigint_handler(sig, frame):
            node.get_logger().info('Ctrl-C detected, shutting down...')
            node.destroy_node()
            rclpy.try_shutdown()

        signal.signal(signal.SIGINT, sigint_handler)

        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()