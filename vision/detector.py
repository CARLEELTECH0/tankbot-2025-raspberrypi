"""
Tankbot Object Detector & Center Error Pipeline
Ported from Hiwonder ArmPi community vision stack:
- /Functions/ColorTracking.py
- /LABConfig.py
- /ArmIK/Transform.py

Extracts rotated bounding boxes, calculates pixel deviation error (ex, ey)
relative to the camera optical axis, and projects into 3D world space (X, Y, Z).
"""

import math
import logging
from dataclasses import dataclass, field, asdict
from typing import Optional, Tuple, List, Dict
import numpy as np
import cv2

logger = logging.getLogger("Tankbot.Vision.Detector")

# Hiwonder ArmPi LAB Thresholds
COLOR_THRESHOLDS_LAB = {
    "red": ((0, 151, 100), (255, 255, 255)),
    "green": ((0, 0, 0), (255, 115, 255)),
    "blue": ((0, 0, 0), (255, 255, 110)),
    "black": ((0, 0, 0), (56, 255, 255)),
    "white": ((193, 0, 0), (255, 250, 255)),
}

# Standard HSV Fallback Thresholds
COLOR_THRESHOLDS_HSV = {
    "red": [
        ((0, 100, 70), (10, 255, 255)),
        ((165, 100, 70), (180, 255, 255))
    ],
    "green": [
        ((35, 70, 70), (85, 255, 255))
    ],
    "blue": [
        ((100, 80, 70), (135, 255, 255))
    ]
}


@dataclass
class VisionTarget:
    """Perception data extracted from a single frame"""
    detected: bool = False
    color: str = "none"
    center_pixel: Tuple[int, int] = (0, 0)
    
    # Error deviation relative to camera optical axis (center of image)
    error_x: float = 0.0          # pixels: positive = right, negative = left
    error_y: float = 0.0          # pixels: positive = up/ahead, negative = down
    error_norm_x: float = 0.0     # normalized [-1.0, 1.0]
    error_norm_y: float = 0.0     # normalized [-1.0, 1.0]

    # 3D Arm Ground Plane Coordinates (cm)
    world_x: float = 0.0          # cm right(+) / left(-)
    world_y: float = 0.0          # cm forward from arm base
    world_z: float = 2.0          # cm table height
    rotation_angle: float = 0.0   # degrees (-90 to +90)
    wrist_pulse: int = 500        # aligned wrist pulse (0-1000)

    bounding_box: List[List[int]] = field(default_factory=list)
    area: float = 0.0
    status: str = "SEARCHING"

    def to_dict(self) -> dict:
        return asdict(self)


class ObjectDetector:
    """
    OpenCV color & geometric detector for 640x480 @ 30 FPS stream.
    Computes center deviation errors and 3D spatial coordinates.
    """

    def __init__(
        self,
        target_color: str = "red",
        color_space: str = "LAB",
        resolution: Tuple[int, int] = (640, 480),
        map_param: float = 0.05,            # cm per pixel
        image_center_distance: float = 20.0  # cm from arm base to camera center
    ):
        self.target_color = target_color.lower()
        self.color_space = color_space.upper()
        self.width = resolution[0]
        self.height = resolution[1]
        self.cx_axis = self.width / 2.0
        self.cy_axis = self.height / 2.0

        self.map_param = map_param
        self.image_center_distance = image_center_distance
        self.kernel = np.ones((6, 6), np.uint8)

        self.min_area = 300.0
        self.trigger_area = 1800.0

    def set_target_color(self, color: str):
        self.target_color = color.lower()

    def detect(self, frame: np.ndarray, annotate: bool = True) -> Tuple[np.ndarray, VisionTarget]:
        """
        Executes color segmentation, contour finding, and error calculation.
        """
        if frame is None or self.target_color in ["none", "off"]:
            return frame, VisionTarget(status="IDLE")

        h, w = frame.shape[:2]
        output_frame = frame.copy() if annotate else frame

        # 1. Blur
        blurred = cv2.GaussianBlur(frame, (11, 11), 11)

        # 2. Thresholding
        mask = self._get_mask(blurred)
        if mask is None:
            return output_frame, VisionTarget(detected=False, color=self.target_color, status="SEARCHING")

        # 3. Morphological cleanup
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, self.kernel)

        # 4. Contour extraction
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        best_contour, best_area = self._find_largest_contour(contours)

        target = VisionTarget(
            detected=False,
            color=self.target_color,
            status="SEARCHING"
        )

        if best_contour is not None and best_area >= self.trigger_area:
            # 5. Rotated Bounding Box
            rect = cv2.minAreaRect(best_contour)
            (cx, cy), (bw, bh), angle = rect
            box_points = np.intp(cv2.boxPoints(rect))

            if bw < bh:
                angle += 90.0

            # 6. Center Error Deviation relative to Optical Axis
            error_x = cx - self.cx_axis
            error_y = self.cy_axis - cy
            norm_ex = error_x / self.cx_axis
            norm_ey = error_y / self.cy_axis

            # 7. World 3D Coordinates
            norm_x = (cx / self.width) * 640.0
            norm_y = (cy / self.height) * 480.0
            world_x = round((norm_x - 320.0) * self.map_param, 2)
            world_y = round((240.0 - norm_y) * self.map_param + self.image_center_distance, 2)
            world_z = 2.0

            # 8. Wrist alignment calculation
            wrist_pulse = self._calc_wrist_pulse(world_x, world_y, angle)

            status = "LOCKED" if (abs(norm_ex) < 0.15 and abs(norm_ey) < 0.15) else "TRACKING"

            target = VisionTarget(
                detected=True,
                color=self.target_color,
                center_pixel=(int(round(cx)), int(round(cy))),
                error_x=round(error_x, 1),
                error_y=round(error_y, 1),
                error_norm_x=round(norm_ex, 3),
                error_norm_y=round(norm_ey, 3),
                world_x=world_x,
                world_y=world_y,
                world_z=world_z,
                rotation_angle=round(angle, 1),
                wrist_pulse=wrist_pulse,
                bounding_box=box_points.tolist(),
                area=round(best_area, 1),
                status=status
            )

            # 9. Annotations
            if annotate:
                color_bgr = (0, 0, 255) if self.target_color == "red" else ((0, 255, 0) if self.target_color == "green" else (255, 120, 0))
                cv2.drawContours(output_frame, [box_points], -1, color_bgr, 2)
                cv2.circle(output_frame, (int(round(cx)), int(round(cy))), 6, (0, 255, 255), -1)

                # Line connecting center of image to target
                cv2.arrowedLine(
                    output_frame,
                    (int(self.cx_axis), int(self.cy_axis)),
                    (int(round(cx)), int(round(cy))),
                    (0, 255, 255),
                    2,
                    tipLength=0.2
                )

                # HUD text
                min_y = min(p[1] for p in box_points)
                cv2.putText(
                    output_frame,
                    f"TARGET: {self.target_color.upper()} ({status})",
                    (int(cx) - 50, max(20, min_y - 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color_bgr,
                    2
                )
                cv2.putText(
                    output_frame,
                    f"Err: ({error_x:+.0f}px, {error_y:+.0f}px) | W: ({world_x:+.1f}cm, {world_y:.1f}cm)",
                    (int(cx) - 80, max(38, min_y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (255, 255, 255),
                    1
                )

        if annotate:
            # Optical Crosshairs
            cv2.line(output_frame, (0, int(self.cy_axis)), (w, int(self.cy_axis)), (80, 80, 80), 1)
            cv2.line(output_frame, (int(self.cx_axis), 0), (int(self.cx_axis), h), (80, 80, 80), 1)
            cv2.circle(output_frame, (int(self.cx_axis), int(self.cy_axis)), 20, (80, 80, 80), 1)

            # Status watermark
            watermark_color = (0, 255, 0) if target.detected else (0, 165, 255)
            cv2.putText(
                output_frame,
                f"SEE-THINK-ACT PIPELINE | TARGET: {self.target_color.upper()} | STATUS: {target.status}",
                (12, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                watermark_color,
                1
            )

        return output_frame, target

    def _get_mask(self, blurred_bgr: np.ndarray) -> Optional[np.ndarray]:
        if self.color_space == "LAB":
            thresholds = COLOR_THRESHOLDS_LAB.get(self.target_color)
            if thresholds is None:
                return None
            lab = cv2.cvtColor(blurred_bgr, cv2.COLOR_BGR2LAB)
            lower, upper = thresholds
            return cv2.inRange(lab, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        else:
            ranges = COLOR_THRESHOLDS_HSV.get(self.target_color)
            if ranges is None:
                return None
            hsv = cv2.cvtColor(blurred_bgr, cv2.COLOR_BGR2HSV)
            combined = None
            for lower, upper in ranges:
                mask_p = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
                combined = mask_p if combined is None else cv2.bitwise_or(combined, mask_p)
            return combined

    def _find_largest_contour(self, contours) -> Tuple[Optional[np.ndarray], float]:
        max_a = 0.0
        best_c = None
        for c in contours:
            a = math.fabs(cv2.contourArea(c))
            if a > max_a and a >= self.min_area:
                max_a = a
                best_c = c
        return best_c, max_a

    def _calc_wrist_pulse(self, x: float, y: float, angle_deg: float) -> int:
        theta_base = round(math.degrees(math.atan2(abs(x), abs(y) if y != 0 else 0.001)), 1)
        angle = abs(angle_deg)

        if x < 0:
            angle1 = (theta_base - angle) if y >= 0 else -(90 + theta_base - angle)
        else:
            angle1 = (90 - theta_base - angle) if y >= 0 else (theta_base + angle)

        angle2 = angle1 - 90 if angle1 > 0 else angle1 + 90
        chosen_angle = angle1 if abs(angle1) < abs(angle2) else angle2
        pulse = int(500 + round(chosen_angle * (1000.0 / 240.0)))
        return max(0, min(1000, pulse))
