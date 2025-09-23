#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from robot_interfaces.msg import HandState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import signal
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from scipy.spatial.transform import Rotation as R

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

                # Get position from wrist
                wrist = hand_landmarks.landmark[0]
                position = Point(x=wrist.x, y=wrist.y, z=0.0)
                
                # Calculate orientation
                orientation = self._calculate_hand_orientation(hand_landmarks)
                
                is_closed = self._is_hand_closed(hand_landmarks)

                landmark_color = (0, 0, 255) if is_closed else (0, 255, 0)
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                    mp.solutions.drawing_styles.get_default_hand_connections_style())

                # Draw orientation axes
                self._draw_orientation_axes(annotated_image, hand_landmarks, wrist)
                
                text_x = int(wrist.x * frame.shape[1])
                text_y = int(wrist.y * frame.shape[0]) - 20
                cv2.putText(annotated_image,
                            f"{handedness} {'CLOSED' if is_closed else 'OPEN'}",
                            (text_x, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, landmark_color, 2)

                detected_hands[handedness] = {
                    "position": position,
                    "orientation": orientation,
                    "is_closed": is_closed,
                    "landmarks": hand_landmarks
                }

        return detected_hands, annotated_image, results.multi_hand_landmarks, results.multi_handedness
    
    def _calculate_hand_orientation(self, landmarks):
        """
        Calculate hand orientation using palm landmarks to create a coordinate system
        Returns: Quaternion representing hand orientation
        """
        # Palm coordinate system:
        # Origin: Wrist (landmark 0)
        # X-axis: From wrist to middle finger MCP (landmark 9)
        # Y-axis: Normal to palm plane (using wrist, index MCP, pinky MCP)
        # Z-axis: Cross product of X and Y
        
        # Get key landmarks
        wrist = np.array([landmarks.landmark[0].x, landmarks.landmark[0].y, landmarks.landmark[0].z])
        middle_mcp = np.array([landmarks.landmark[9].x, landmarks.landmark[9].y, landmarks.landmark[9].z])
        index_mcp = np.array([landmarks.landmark[5].x, landmarks.landmark[5].y, landmarks.landmark[5].z])
        pinky_mcp = np.array([landmarks.landmark[17].x, landmarks.landmark[17].y, landmarks.landmark[17].z])
        
        # Calculate vectors
        x_axis = middle_mcp - wrist  # Forward direction (palm normal)
        vec1 = index_mcp - wrist
        vec2 = pinky_mcp - wrist
        
        # Calculate normal vector to palm (Y-axis)
        y_axis = np.cross(vec1, vec2)
        
        # Ensure vectors are not zero
        if np.linalg.norm(x_axis) < 1e-6 or np.linalg.norm(y_axis) < 1e-6:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Default orientation
        
        # Normalize axes
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # Calculate Z-axis (right-hand rule)
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)
        
        # Re-calculate Y-axis to ensure orthogonality
        y_axis = np.cross(z_axis, x_axis)
        
        # Create rotation matrix
        rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
        
        # Convert to quaternion
        try:
            rotation = R.from_matrix(rotation_matrix)
            quat = rotation.as_quat()  # Returns [x, y, z, w]
            return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        except:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    
    def _draw_orientation_axes(self, image, landmarks, wrist, length=50):
        """Draw orientation axes on the image for visualization"""
        h, w = image.shape[:2]
        
        # Convert normalized coordinates to pixel coordinates
        wrist_px = (int(wrist.x * w), int(wrist.y * h))
        
        # Calculate direction vectors (simplified 2D projection)
        middle_mcp = landmarks.landmark[9]
        index_mcp = landmarks.landmark[5]
        
        # Forward direction (X-axis - red)
        forward_dir = np.array([middle_mcp.x - wrist.x, middle_mcp.y - wrist.y])
        if np.linalg.norm(forward_dir) > 0.01:
            forward_dir = forward_dir / np.linalg.norm(forward_dir)
            end_x = int((wrist.x + forward_dir[0] * 0.1) * w)
            end_y = int((wrist.y + forward_dir[1] * 0.1) * h)
            cv2.arrowedLine(image, wrist_px, (end_x, end_y), (0, 0, 255), 3)
        
        # Right direction approximation (Z-axis - blue)
        right_dir = np.array([index_mcp.x - wrist.x, index_mcp.y - wrist.y])
        if np.linalg.norm(right_dir) > 0.01:
            # Project to perpendicular direction
            right_dir_perp = np.array([-right_dir[1], right_dir[0]])
            right_dir_perp = right_dir_perp / np.linalg.norm(right_dir_perp)
            end_x = int((wrist.x + right_dir_perp[0] * 0.05) * w)
            end_y = int((wrist.y + right_dir_perp[1] * 0.05) * h)
            cv2.arrowedLine(image, wrist_px, (end_x, end_y), (255, 0, 0), 2)
    
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

        # Initialize tracking variables
        self.initial_positions = {"left": None, "right": None}
        self.initial_orientations = {"left": None, "right": None}
        self.initial_hand_sizes = {"left": None, "right": None}
        self.latest_detected_hands = {"left": None, "right": None}

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video device")
            raise RuntimeError("Could not open video device")

        self.get_logger().info('Hand Tracking Node started with orientation tracking')

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

        for hand_type in ['left', 'right']:
            msg = HandState()
            msg.detected = detected_hands[hand_type] is not None
            
            # Set hand_type using enum values - FIXED
            if hand_type == "left":
                msg.hand_type = HandState.HAND_TYPE_LEFT
            else:  # hand_type == "right"
                msg.hand_type = HandState.HAND_TYPE_RIGHT

            if msg.detected:
                hand_info = detected_hands[hand_type]
                raw_position = hand_info["position"]
                raw_orientation = hand_info["orientation"]
                msg.is_closed = hand_info["is_closed"]

                # Set initial references
                if self.initial_hand_sizes[hand_type] is None:
                    # First appearance - set initial references
                    for j, classification in enumerate(handedness_list):
                        if classification.classification[0].label.lower() == hand_type:
                            hand_landmarks = landmarks_list[j]
                            hand_size = self.detector.compute_hand_size(hand_landmarks)
                            self.initial_hand_sizes[hand_type] = hand_size
                            self.initial_positions[hand_type] = (raw_position.x, raw_position.y)
                            self.initial_orientations[hand_type] = raw_orientation
                            
                            # Set relative values to zero
                            msg.position.x = 0.0
                            msg.position.y = 0.0
                            msg.position.z = 0.0
                            msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                            break
                else:
                    # Calculate relative position
                    scale_x, scale_y, scale_z = 0.5, 0.5, 0.1
                    origin_x, origin_y = self.initial_positions[hand_type]
                    msg.position.x = (raw_position.x - origin_x) * scale_x
                    msg.position.y = (raw_position.y - origin_y) * scale_y

                    # Calculate relative orientation
                    relative_orientation = self._calculate_relative_orientation(
                        self.initial_orientations[hand_type], raw_orientation)
                    msg.orientation = relative_orientation

                    # Calculate Z from hand size
                    for j, classification in enumerate(handedness_list):
                        if classification.classification[0].label.lower() == hand_type:
                            hand_landmarks = landmarks_list[j]
                            hand_size = self.detector.compute_hand_size(hand_landmarks)
                            msg.position.z = (hand_size / self.initial_hand_sizes[hand_type]) * scale_z
                            break
            else:
                # Hand not detected - reset values
                msg.position.x = 0.0
                msg.position.y = 0.0
                msg.position.z = 0.0
                msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                msg.is_closed = False  # Explicitly set when not detected
                self.initial_hand_sizes[hand_type] = None
                self.initial_positions[hand_type] = None
                self.initial_orientations[hand_type] = None

            # Log state changes
            if msg.detected != self.prev_hand_presence[hand_type]:
                if msg.detected:
                    self.get_logger().info(
                        f"{hand_type.capitalize()} hand appeared - "
                        f"Pos: ({msg.position.x:.2f}, {msg.position.y:.2f}, z={msg.position.z:.2f}) - "
                        f"Orientation: ({msg.orientation.x:.2f}, {msg.orientation.y:.2f}, "
                        f"{msg.orientation.z:.2f}, {msg.orientation.w:.2f}) - "
                        f"{'CLOSED' if msg.is_closed else 'OPEN'}"
                    )
                else:
                    self.get_logger().info(f"{hand_type.capitalize()} hand disappeared")
                self.prev_hand_presence[hand_type] = msg.detected

            self.hand_publishers[hand_type].publish(msg)

    def _calculate_relative_orientation(self, initial_quat, current_quat):
        """Calculate relative orientation from initial to current"""
        # Convert to scipy Rotation objects
        initial_rot = R.from_quat([initial_quat.x, initial_quat.y, initial_quat.z, initial_quat.w])
        current_rot = R.from_quat([current_quat.x, current_quat.y, current_quat.z, current_quat.w])
        
        # Calculate relative rotation: relative = current * initial^-1
        relative_rot = current_rot * initial_rot.inv()
        relative_quat = relative_rot.as_quat()
        
        return Quaternion(
            x=relative_quat[0],
            y=relative_quat[1], 
            z=relative_quat[2],
            w=relative_quat[3]
        )

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