"""
Tankbot Color Tracker & Target Coordinate Extraction Pipeline
Ported and adapted from Hiwonder ArmPi community vision stack:
- /Functions/ColorTracking.py
- /LABConfig.py
- /ArmIK/Transform.py

Implements LAB/HSV color space thresholding, morphological noise suppression,
contour area filtering, minAreaRect rotated bounding boxes, pixel-to-world
spatial transformation, and gripper alignment angle calculation.
"""

import math
import logging
from dataclasses import dataclass, field, asdict
from typing import Optional, Tuple, List, Dict
import numpy as np

logger = logging.getLogger("Tankbot.Vision.Tracker")


# Exact LAB Color Thresholds from Hiwonder ArmPi LABConfig.py
ARMPI_LAB_THRESHOLDS = {
    "red": ((0, 151, 100), (255, 255, 255)),
    "green": ((0, 0, 0), (255, 115, 255)),
    "blue": ((0, 0, 0), (255, 255, 110)),
    "black": ((0, 0, 0), (56, 255, 255)),
    "white": ((193, 0, 0), (255, 250, 255)),
}

# Standard HSV Fallback Thresholds (robust under varying daylight)
ARMPI_HSV_THRESHOLDS = {
    "red": [
        ((0, 100, 60), (10, 255, 255)),
        ((165, 100, 60), (180, 255, 255))
    ],
    "green": [
        ((35, 60, 60), (85, 255, 255))
    ],
    "blue": [
        ((100, 80, 60), (135, 255, 255))
    ]
}

# Display BGR colors for annotations
COLOR_BGR = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 120, 0),
    "black": (50, 50, 50),
    "white": (240, 240, 240),
    "target": (0, 255, 255)
}


@dataclass
class DetectionResult:
    """Standardized output data structure from ColorTracker"""
    detected: bool = False
    color: str = "none"
    center_pixel: Tuple[int, int] = (0, 0)
    world_x: float = 0.0          # cm relative to arm base center (+right, -left)
    world_y: float = 0.0          # cm forward from arm base center
    world_z: float = 2.0          # cm table height offset
    rotation_angle: float = 0.0   # Object orientation angle (-90 to +90 deg)
    wrist_servo_pulse: int = 500  # Gripper alignment pulse (0-1000)
    bounding_box: List[List[int]] = field(default_factory=list)
    area: float = 0.0
    status: str = "SEARCHING"     # "SEARCHING", "LOCKED", "TRACKING", "IDLE"

    def to_dict(self) -> dict:
        return asdict(self)


class ColorTracker:
    """
    Computer Vision pipeline for color segmentation and coordinate extraction.
    """

    def __init__(
        self,
        target_color: str = "red",
        color_space: str = "LAB",  # "LAB" or "HSV"
        resolution: Tuple[int, int] = (640, 480),
        min_contour_area: float = 300.0,
        trigger_contour_area: float = 1800.0,
        map_param: float = 0.05,            # cm per pixel
        image_center_distance: float = 20.0  # cm from arm base to camera center
    ):
        self.target_color = target_color.lower()
        self.color_space = color_space.upper()
        self.width = resolution[0]
        self.height = resolution[1]
        self.min_contour_area = min_contour_area
        self.trigger_contour_area = trigger_contour_area
        self.map_param = map_param
        self.image_center_distance = image_center_distance

        self.morph_kernel = np.ones((6, 6), np.uint8)
        self.last_result = DetectionResult()
        logger.info(f"ColorTracker initialized for target '{self.target_color}' using {self.color_space} color space.")

    def set_target_color(self, color: str):
        """Switches active tracking target color ('red', 'green', 'blue', 'off'/'none')"""
        valid_colors = ["red", "green", "blue", "none", "off"]
        c = color.lower()
        if c in valid_colors:
            self.target_color = c
            logger.info(f"ColorTracker target set to: {c}")

    def process_frame(self, frame: np.ndarray, annotate: bool = True) -> Tuple[np.ndarray, DetectionResult]:
        """
        Executes the vision pipeline on a single BGR image.
        Returns:
            annotated_frame: Image with bounding boxes, center point, and HUD
            result: DetectionResult containing pixel & world target coordinates
        """
        if frame is None or self.target_color in ["none", "off"]:
            self.last_result = DetectionResult(status="IDLE")
            return frame, self.last_result

        h, w = frame.shape[:2]
        output_frame = frame.copy() if annotate else frame

        try:
            import cv2
        except ImportError:
            # Fallback without OpenCV: returns empty detection
            return output_frame, DetectionResult(status="NO_OPENCV")

        # 1. Gaussian Blur to reduce sensor noise (11x11, sigma=11)
        blurred = cv2.GaussianBlur(frame, (11, 11), 11)

        # 2. Color Space Conversion & Thresholding
        mask = self._generate_color_mask(blurred, self.target_color)
        if mask is None:
            self.last_result = DetectionResult(detected=False, color=self.target_color, status="SEARCHING")
            return output_frame, self.last_result

        # 3. Morphological Opening (remove noise) & Closing (close holes)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, self.morph_kernel)

        # 4. Find External Contours
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        best_contour, best_area = self._get_area_max_contour(contours)

        result = DetectionResult(
            detected=False,
            color=self.target_color,
            status="SEARCHING"
        )

        if best_contour is not None and best_area > self.trigger_contour_area:
            # 5. MinAreaRect Bounding Box & Center Point Extraction
            rect = cv2.minAreaRect(best_contour)
            (cx, cy), (box_w, box_h), rotation_angle = rect
            box_points = np.intp(cv2.boxPoints(rect))

            # Normalize rotation angle
            if box_w < box_h:
                rotation_angle = rotation_angle + 90.0

            # 6. Transform Image Pixel Coordinates -> Real-World 3D Robot Base Coordinates (cm)
            world_x, world_y = self.pixel_to_world(cx, cy)
            world_z = 2.0  # standard block pickup height

            # 7. Calculate Wrist Roll Servo Alignment Angle
            wrist_pulse = self.calculate_wrist_servo_pulse(world_x, world_y, rotation_angle)

            result = DetectionResult(
                detected=True,
                color=self.target_color,
                center_pixel=(int(round(cx)), int(round(cy))),
                world_x=world_x,
                world_y=world_y,
                world_z=world_z,
                rotation_angle=round(rotation_angle, 1),
                wrist_servo_pulse=wrist_pulse,
                bounding_box=box_points.tolist(),
                area=round(best_area, 1),
                status="LOCKED" if best_area > 3500 else "TRACKING"
            )

            # 8. Annotate Output Frame
            if annotate:
                draw_color = COLOR_BGR.get(self.target_color, (0, 255, 255))
                # Rotated bounding box
                cv2.drawContours(output_frame, [box_points], -1, draw_color, 2)
                # Center point marker
                cv2.circle(output_frame, (int(round(cx)), int(round(cy))), 5, (0, 255, 255), -1)
                # Target HUD text
                coord_str = f"({world_x:+.1f}cm, {world_y:+.1f}cm) | {rotation_angle:+.0f}deg"
                min_y = min(p[1] for p in box_points)
                cv2.putText(
                    output_frame,
                    f"TARGET [{self.target_color.upper()}]",
                    (int(cx) - 40, max(20, min_y - 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    draw_color,
                    2
                )
                cv2.putText(
                    output_frame,
                    coord_str,
                    (int(cx) - 60, max(38, min_y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1
                )

        if annotate:
            # Draw center crosshairs on camera frame
            cv2.line(output_frame, (0, h // 2), (w, h // 2), (100, 100, 100), 1)
            cv2.line(output_frame, (w // 2, 0), (w // 2, h), (100, 100, 100), 1)
            # Top status banner
            status_color = (0, 255, 0) if result.detected else (0, 165, 255)
            cv2.putText(
                output_frame,
                f"CV MODE: {self.color_space} | TARGET: {self.target_color.upper()} | STATUS: {result.status}",
                (15, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                status_color,
                1
            )

        self.last_result = result
        return output_frame, result

    def _generate_color_mask(self, blurred_frame: np.ndarray, color_name: str) -> Optional[np.ndarray]:
        """Applies LAB or HSV thresholding to extract binary mask"""
        import cv2

        if self.color_space == "LAB":
            thresholds = ARMPI_LAB_THRESHOLDS.get(color_name)
            if thresholds is None:
                return None
            lab_img = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2LAB)
            lower, upper = thresholds
            mask = cv2.inRange(lab_img, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
            return mask

        elif self.color_space == "HSV":
            threshold_ranges = ARMPI_HSV_THRESHOLDS.get(color_name)
            if threshold_ranges is None:
                return None
            hsv_img = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
            combined_mask = None
            for lower, upper in threshold_ranges:
                mask_part = cv2.inRange(hsv_img, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
                combined_mask = mask_part if combined_mask is None else cv2.bitwise_or(combined_mask, mask_part)
            return combined_mask

        return None

    def _get_area_max_contour(self, contours) -> Tuple[Optional[np.ndarray], float]:
        """Finds the contour with the largest area exceeding min_contour_area threshold"""
        import cv2
        max_area = 0.0
        best_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > max_area and area >= self.min_contour_area:
                max_area = area
                best_contour = c
        return best_contour, max_area

    def pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        """
        Converts 2D pixel coordinates (640x480) into 3D robot ground coordinates (cm).
        Matches Hiwonder ArmPi Transform.convertCoordinate():
        - Origin (0,0): Projection of the arm base turntable onto the ground
        - +X: Right, -X: Left
        - +Y: Forward from arm base
        """
        # Linear normalization to 640x480 standard grid
        norm_x = (px / self.width) * 640.0
        norm_y = (py / self.height) * 480.0

        # Offset from optical center
        dx = norm_x - 320.0
        world_x = round(dx * self.map_param, 2)

        dy = 240.0 - norm_y
        world_y = round(dy * self.map_param + self.image_center_distance, 2)

        return world_x, world_y

    def calculate_wrist_servo_pulse(self, x: float, y: float, angle_deg: float) -> int:
        """
        Calculates wrist roll servo pulse (Tankbot Servo ID 5 / ArmPi Servo ID 2)
        to align the gripper claw parallel to the block's detected orientation.
        Ported from Hiwonder ArmPi Transform.getAngle(x, y, angle).
        """
        theta_base = round(math.degrees(math.atan2(abs(x), abs(y) if y != 0 else 0.001)), 1)
        angle = abs(angle_deg)

        if x < 0:
            angle1 = (theta_base - angle) if y >= 0 else -(90 + theta_base - angle)
        else:
            angle1 = (90 - theta_base - angle) if y >= 0 else (theta_base + angle)

        angle2 = angle1 - 90 if angle1 > 0 else angle1 + 90
        chosen_angle = angle1 if abs(angle1) < abs(angle2) else angle2

        # Map angle to 0-1000 pulse (240 deg range, centered at 500)
        pulse = int(500 + round(chosen_angle * (1000.0 / 240.0)))
        return max(0, min(1000, pulse))
