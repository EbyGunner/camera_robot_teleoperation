import cv2
import os
import sys

library_path = os.path.join(os.path.join(os.path.dirname(__file__), 'hand_tracking'), 'external_libraries')

print(library_path)

sys.path.insert(1, library_path)

import mediapipe as mp

class HandTracker:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.8)
        self.mp_drawing = mp.solutions.drawing_utils
        self.hand_states = {}
        self.prev_hand_count = 0

    def calculate_finger_state(self, landmarks):
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
        
        return [thumb_state] + finger_states

    def detect_fist(self, finger_states):
        return sum(finger_states) < 2

    def get_hand_position(self, landmarks, frame_width, frame_height):
        """Returns the (x,y) position in pixel coordinates"""
        wrist = landmarks.landmark[0]
        return (
            int(wrist.x * frame_width),
            int(wrist.y * frame_height)
        )

    def process_frame(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        
        # Reset detection flags
        for hand_id in list(self.hand_states.keys()):
            self.hand_states[hand_id]["detected"] = False
        
        current_hands = []
        frame_height, frame_width = frame.shape[:2]
        
        if results.multi_hand_landmarks:
            for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                handedness = results.multi_handedness[i].classification[0].label
                hand_id = f"{handedness}_{i}"
                current_hands.append(hand_id)
                
                # Calculate finger states and gesture
                finger_states = self.calculate_finger_state(hand_landmarks)
                is_fist = self.detect_fist(finger_states)
                gesture = "FIST" if is_fist else "OPEN"
                
                # Get hand position in pixels
                pos_x, pos_y = self.get_hand_position(hand_landmarks, frame_width, frame_height)
                
                # Update hand state
                if hand_id not in self.hand_states:
                    print(f"New hand detected: {hand_id} at ({pos_x}, {pos_y})")
                    self.hand_states[hand_id] = {
                        "landmarks": None,
                        "gesture": "UNKNOWN",
                        "position": (0, 0),
                        "first_detected": cv2.getTickCount(),
                        "detected": True
                    }
                
                self.hand_states[hand_id].update({
                    "landmarks": hand_landmarks,
                    "gesture": gesture,
                    "position": (pos_x, pos_y),
                    "detected": True,
                    "last_detected": cv2.getTickCount(),
                    "finger_states": finger_states
                })
                
                # Draw landmarks with different colors for different states
                landmark_color = (0, 0, 255) if is_fist else (0, 255, 0)
                self.mp_drawing.draw_landmarks(
                    frame, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS,
                    mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                    mp.solutions.drawing_styles.get_default_hand_connections_style())
                
                # Display hand info with position
                info_text = f"{handedness} {gesture} ({pos_x}, {pos_y})"
                cv2.putText(frame, info_text, (pos_x - 50, pos_y - 20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, landmark_color, 2)
                
                # Draw position marker
                cv2.circle(frame, (pos_x, pos_y), 8, landmark_color, -1)
                cv2.circle(frame, (pos_x, pos_y), 10, (255, 255, 255), 2)
        
        # Check for disappeared hands
        disappeared_hands = [hid for hid in self.hand_states if hid not in current_hands]
        for hand_id in disappeared_hands:
            print(f"Hand lost: {hand_id}")
            del self.hand_states[hand_id]
        
        # Display summary information
        summary_y = 30
        cv2.putText(frame, f"Hands detected: {len(current_hands)}", (10, summary_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        summary_y += 30
        
        # Display each hand's status
        for i, hand_id in enumerate(current_hands):
            state = self.hand_states[hand_id]
            pos_text = f"{hand_id}: {state['gesture']} at {state['position']}"
            cv2.putText(frame, pos_text, (10, summary_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            summary_y += 25
        
        return frame

def main():
    tracker = HandTracker()
    cap = cv2.VideoCapture(0)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Can't receive frame.")
                break
            
            # Flip frame horizontally for more intuitive movement
            frame = cv2.flip(frame, 1)
            frame = tracker.process_frame(frame)
            
            cv2.imshow("Hand Tracking with Position", frame)
            
            if cv2.waitKey(1) == ord('q'):
                break
    finally:
        tracker.hands.close()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()