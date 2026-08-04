"""Mediapipe hand detection wrapper.

Owns everything mediapipe- and OpenCV-specific; emits plain-numpy
``HandObservation`` objects that the geometry and tracking modules (and
tests) can consume without touching mediapipe.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

from . import hand_geometry
from .mediapipe_loader import load_mediapipe

mp = load_mediapipe()


@dataclass
class HandObservation:
    position_xy: np.ndarray      # wrist, normalised image coords [x, y]
    orientation: np.ndarray      # palm quaternion [x, y, z, w]
    size: float                  # apparent hand size
    is_closed: bool


class HandDetector:
    def __init__(self, max_num_hands: int = 2,
                 min_detection_confidence: float = 0.8,
                 min_tracking_confidence: float = 0.8):
        self._mp_hands = mp.solutions.hands
        self._hands = self._mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence)
        self._drawing = mp.solutions.drawing_utils
        self._styles = mp.solutions.drawing_styles

    # ------------------------------------------------------------------
    def detect(self, frame_bgr) -> Tuple[Dict[str, Optional[HandObservation]], 'np.ndarray']:
        """Returns ({'left': obs|None, 'right': obs|None}, annotated_frame)."""
        results = self._hands.process(
            cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))

        observations: Dict[str, Optional[HandObservation]] = {
            'left': None, 'right': None}
        annotated = frame_bgr.copy()

        if not results.multi_hand_landmarks:
            return observations, annotated

        for landmarks, handedness in zip(results.multi_hand_landmarks,
                                         results.multi_handedness):
            label = handedness.classification[0].label.lower()
            if observations.get(label) is not None:
                continue                        # ignore duplicate hands

            lm = np.array([[p.x, p.y, p.z] for p in landmarks.landmark])
            obs = HandObservation(
                position_xy=lm[0, :2].copy(),
                orientation=hand_geometry.orientation_quat(lm),
                size=hand_geometry.hand_size(lm),
                is_closed=hand_geometry.is_closed(lm))
            observations[label] = obs
            self._annotate(annotated, landmarks, label, obs)

        return observations, annotated

    # ------------------------------------------------------------------
    def _annotate(self, image, landmarks, label: str,
                  obs: HandObservation) -> None:
        color = (0, 0, 255) if obs.is_closed else (0, 255, 0)
        self._drawing.draw_landmarks(
            image, landmarks, self._mp_hands.HAND_CONNECTIONS,
            self._styles.get_default_hand_landmarks_style(),
            self._styles.get_default_hand_connections_style())
        h, w = image.shape[:2]
        text_pos = (int(obs.position_xy[0] * w),
                    max(int(obs.position_xy[1] * h) - 20, 15))
        cv2.putText(image,
                    f"{label} {'CLOSED' if obs.is_closed else 'OPEN'}",
                    text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    def close(self) -> None:
        self._hands.close()
