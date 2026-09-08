"""
Tankbot Inverse Kinematics (IK) Bridge
Ported from Hiwonder ArmPi community stack:
- /ArmIK/InverseKinematics.py
- /ArmIK/ArmMoveIK.py
- /ArmIK/Transform.py

Converts target (X, Y, Z) real-world coordinates from Computer Vision
into joint angles and generates target pulse dictionaries {ID: pulse}
compatible with Tankbot's ServoController.
"""

import math
import logging
from typing import Optional, Dict, Tuple, Union
import numpy as np

logger = logging.getLogger("Tankbot.Vision.IK")


class ArmIK:
    """
    4-DOF Analytical Geometric Inverse Kinematics Solver.
    Solves for Base Rotation, Shoulder, Elbow, and Wrist Pitch angles.
    """

    # Link lengths in centimeters (standard Hiwonder 6-DOF / ArmPi arm geometry)
    L1 = 6.85    # Turntable ground base to shoulder pitch axis (cm)
    L2 = 10.16   # Shoulder axis to elbow axis (cm)
    L3 = 9.64    # Elbow axis to wrist pitch axis (cm)
    L4 = 16.50   # Wrist pitch axis to tip of closed gripper (cm)

    def __init__(self, l1: float = L1, l2: float = L2, l3: float = L3, l4: float = L4):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

    def solve(self, coordinate: Tuple[float, float, float], alpha: float = -90.0) -> Optional[Dict[str, float]]:
        """
        Calculates joint angles for a given target point (X, Y, Z) in cm
        and gripper approach angle alpha in degrees (-90 = pointing straight down).
        Returns dictionary of joint angles {theta3, theta4, theta5, theta6} or None if unreachable.
        """
        x, y, z = coordinate

        # 1. Base Rotation Angle (theta6) in degrees
        # In ground plane, atan2(Y, X) gives orientation from X axis (90 deg is forward)
        theta6 = math.degrees(math.atan2(y, x if x != 0 else 0.0001))

        # 2. Projected 2D ground distance from base center to target
        p_o = math.sqrt(x * x + y * y)

        # 3. Wrist position relative to base origin
        alpha_rad = math.radians(alpha)
        cd = self.l4 * math.cos(alpha_rad)
        pd = self.l4 * math.sin(alpha_rad)

        af = p_o - cd
        cf = z - self.l1 - pd
        ac = math.sqrt(af * af + cf * cf)

        # 4. Reachability constraints
        if cf < -self.l1:
            # Below floor boundary
            return None

        if ac > (self.l2 + self.l3) or ac < abs(self.l2 - self.l3):
            # Outside mechanical workspace envelope
            return None

        # 5. Law of Cosines for Elbow (theta4)
        cos_abc = (self.l2 * self.l2 + self.l3 * self.l3 - ac * ac) / (2.0 * self.l2 * self.l3)
        cos_abc = max(-1.0, min(1.0, cos_abc))
        abc = math.acos(cos_abc)
        theta4 = 180.0 - math.degrees(abc)

        # 6. Law of Cosines for Shoulder (theta5)
        cos_bac = (ac * ac + self.l2 * self.l2 - self.l3 * self.l3) / (2.0 * self.l2 * ac)
        cos_bac = max(-1.0, min(1.0, cos_bac))
        bac = math.acos(cos_bac)

        cos_caf = max(-1.0, min(1.0, af / ac))
        caf = math.acos(cos_caf)
        zf_flag = 1.0 if cf >= 0 else -1.0
        theta5 = math.degrees(caf * zf_flag + bac)

        # 7. Wrist Pitch (theta3)
        theta3 = alpha - theta5 + theta4

        return {
            "theta3": theta3,  # Wrist Pitch (deg)
            "theta4": theta4,  # Elbow (deg)
            "theta5": theta5,  # Shoulder (deg)
            "theta6": theta6,  # Base (deg)
            "alpha": alpha
        }

    def solve_with_pitch_range(
        self,
        coordinate: Tuple[float, float, float],
        alpha_target: float = -90.0,
        alpha_min: float = -90.0,
        alpha_max: float = 0.0,
        step: float = 2.0
    ) -> Optional[Dict[str, float]]:
        """
        Attempts to solve at alpha_target; if unreachable, searches pitch angles
        between alpha_min and alpha_max to find the closest reachable pose.
        """
        # First attempt requested angle
        direct_solution = self.solve(coordinate, alpha_target)
        if direct_solution is not None:
            return direct_solution

        # Search around target
        best_solution = None
        min_diff = 999.0

        for a in np.arange(alpha_min, alpha_max + step, step):
            sol = self.solve(coordinate, float(a))
            if sol is not None:
                diff = abs(float(a) - alpha_target)
                if diff < min_diff:
                    min_diff = diff
                    best_solution = sol

        return best_solution


class IKBridge:
    """
    Bridges Computer Vision detections and Tankbot's physical 6-DOF Servo Controller.
    Translates physical joint angles to 0-1000 pulse commands matching Tankbot's joint layout.
    """

    # Tankbot Joint Layout:
    # ID 1: Base Rotation  (center 500 = 90 deg forward)
    # ID 2: Shoulder       (center 500 = 90 deg vertical)
    # ID 3: Elbow          (center 500 = 90 deg bend)
    # ID 4: Wrist Pitch    (center 500 = 0 deg inline)
    # ID 5: Wrist Roll     (center 500 = neutral grip angle)
    # ID 6: Gripper Claw   (open 200, closed 500-600)

    PULSE_PER_DEG = 1000.0 / 240.0  # ~4.1667 pulse / degree for 240-degree LX-16A servos

    def __init__(self, arm_ik: Optional[ArmIK] = None):
        self.arm_ik = arm_ik or ArmIK()

        # Preset placement coordinates for sorted colored cubes (X, Y, Z) in cm
        self.bin_coordinates = {
            "red":   (-14.0, 12.0, 3.0),
            "green": (-14.0, 6.0,  3.0),
            "blue":  (-14.0, 0.0,  3.0),
        }

    def angles_to_pulses(
        self,
        angles: Dict[str, float],
        wrist_roll_pulse: int = 500,
        claw_pulse: int = 500
    ) -> Optional[Dict[int, int]]:
        """
        Maps kinematic joint angles (theta3..theta6) to Tankbot servo IDs (1..6)
        clamped to valid pulse limits (0 to 1000).
        """
        try:
            # 1. Base (ID 1): theta6 = 90 is center forward (pulse 500)
            t6 = angles["theta6"]
            pulse1 = int(round(500 + (t6 - 90.0) * self.PULSE_PER_DEG))

            # 2. Shoulder (ID 2): theta5 = 90 is upright vertical (pulse 500)
            t5 = angles["theta5"]
            pulse2 = int(round(500 - (90.0 - t5) * self.PULSE_PER_DEG))

            # 3. Elbow (ID 3): theta4 = 90 is right-angle bend (pulse 500)
            t4 = angles["theta4"]
            pulse3 = int(round(500 + (t4 - 90.0) * self.PULSE_PER_DEG))

            # 4. Wrist Pitch (ID 4): theta3
            t3 = angles["theta3"]
            pulse4 = int(round(500 + t3 * self.PULSE_PER_DEG))

            # 5. Wrist Roll (ID 5)
            pulse5 = max(0, min(1000, int(wrist_roll_pulse)))

            # 6. Gripper Claw (ID 6)
            pulse6 = max(130, min(875, int(claw_pulse)))

            # Range clamping and safety verification
            def clamp(val, low=0, high=1000):
                return max(low, min(high, val))

            targets = {
                1: clamp(pulse1, 50, 950),   # Base
                2: clamp(pulse2, 50, 950),   # Shoulder
                3: clamp(pulse3, 50, 950),   # Elbow
                4: clamp(pulse4, 50, 950),   # Wrist Pitch
                5: clamp(pulse5, 0, 1000),   # Wrist Roll
                6: clamp(pulse6, 130, 875),  # Claw
            }

            return targets

        except Exception as e:
            logger.error(f"Error converting angles to pulses: {e}")
            return None

    def calculate_target_pose(
        self,
        world_x: float,
        world_y: float,
        world_z: float = 2.0,
        alpha: float = -90.0,
        wrist_roll_pulse: int = 500,
        claw_pulse: int = 500
    ) -> Optional[Dict[int, int]]:
        """
        Solves IK for (X, Y, Z) in cm and returns full 6-joint servo pulse dictionary.
        Returns None if point cannot be reached.
        """
        sol = self.arm_ik.solve_with_pitch_range((world_x, world_y, world_z), alpha_target=alpha)
        if sol is None:
            logger.debug(f"IK unreachable for target: ({world_x}, {world_y}, {world_z})")
            return None

        return self.angles_to_pulses(sol, wrist_roll_pulse=wrist_roll_pulse, claw_pulse=claw_pulse)

    def get_bin_coordinate(self, color: str) -> Tuple[float, float, float]:
        """Returns drop-off tray coordinates for the specified color"""
        return self.bin_coordinates.get(color.lower(), (-14.0, 6.0, 3.0))
