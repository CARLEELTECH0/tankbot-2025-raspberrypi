"""
Tankbot 6-DOF Geometric Inverse Kinematics (IK) Solver
Ported from Hiwonder ArmPi community stack:
- /ArmIK/InverseKinematics.py
- /ArmIK/ArmMoveIK.py
- /ArmIK/Transform.py

Given a 3D target coordinate (X, Y, Z) in cm and optional pitch/roll constraints,
analytically calculates the joint angles and translates them into servo pulse values
(0-1000) for all 6 degrees of freedom.
"""

import math
import logging
from dataclasses import dataclass
from typing import Optional, Dict, Tuple
import numpy as np

logger = logging.getLogger("Tankbot.Kinematics.IK")


@dataclass
class KinematicSolution:
    """Complete solution containing joint angles and hardware pulses"""
    reachable: bool
    coordinate: Tuple[float, float, float]  # (X, Y, Z) in cm
    alpha_pitch: float                      # Gripper approach angle in degrees
    
    # Angles in degrees
    theta_base: float = 90.0
    theta_shoulder: float = 90.0
    theta_elbow: float = 90.0
    theta_wrist_pitch: float = 0.0
    theta_wrist_roll: float = 0.0
    
    # Hardware pulse values (0-1000) for Tankbot Joint IDs 1 to 6
    pulses: Dict[int, int] = None

    def to_dict(self) -> dict:
        return {
            "reachable": self.reachable,
            "coordinate": self.coordinate,
            "alpha_pitch": self.alpha_pitch,
            "angles": {
                "base": self.theta_base,
                "shoulder": self.theta_shoulder,
                "elbow": self.theta_elbow,
                "wrist_pitch": self.theta_wrist_pitch,
                "wrist_roll": self.theta_wrist_roll,
            },
            "pulses": self.pulses or {}
        }


class IKSolver:
    """
    4-DOF Analytical Geometric Solver with 6-DOF Mapping.
    Link lengths (cm):
        L1 = 6.85  (turntable base to shoulder pitch axis)
        L2 = 10.16 (shoulder pitch axis to elbow pitch axis)
        L3 = 9.64  (elbow pitch axis to wrist pitch axis)
        L4 = 16.50 (wrist pitch axis to claw tip)
    """

    L1 = 6.85
    L2 = 10.16
    L3 = 9.64
    L4 = 16.50

    PULSE_PER_DEG = 1000.0 / 240.0  # ~4.1667 pulse / degree (LX-16A bus servos)

    def __init__(self, l1: float = L1, l2: float = L2, l3: float = L3, l4: float = L4):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

    def solve_exact(
        self,
        target_xyz: Tuple[float, float, float],
        alpha_pitch: float = -90.0,
        wrist_roll_pulse: int = 500,
        claw_pulse: int = 500
    ) -> Optional[KinematicSolution]:
        """
        Solves IK for exact (X, Y, Z) in cm at exact approach angle alpha.
        Returns KinematicSolution or None if unreachable at this specific angle.
        """
        x, y, z = target_xyz

        # 1. Base Rotation theta_base (degrees)
        # 90 degrees corresponds to straight ahead (+Y)
        theta_base = math.degrees(math.atan2(y, x if x != 0 else 0.0001))

        # 2. Projected 2D ground distance
        p_o = math.sqrt(x * x + y * y)

        # 3. Wrist position
        alpha_rad = math.radians(alpha_pitch)
        cd = self.l4 * math.cos(alpha_rad)
        pd = self.l4 * math.sin(alpha_rad)

        af = p_o - cd
        cf = z - self.l1 - pd
        ac = math.sqrt(af * af + cf * cf)

        # 4. Boundary & Triangle Inequality Checks
        if cf < -self.l1:
            return None  # Collision below mounting deck

        if ac > (self.l2 + self.l3) or ac < abs(self.l2 - self.l3):
            return None  # Beyond arm reach or inside inner singularity

        # 5. Law of Cosines for Elbow (theta_elbow)
        cos_abc = (self.l2 * self.l2 + self.l3 * self.l3 - ac * ac) / (2.0 * self.l2 * self.l3)
        cos_abc = max(-1.0, min(1.0, cos_abc))
        abc = math.acos(cos_abc)
        theta_elbow = 180.0 - math.degrees(abc)

        # 6. Law of Cosines for Shoulder (theta_shoulder)
        cos_bac = (ac * ac + self.l2 * self.l2 - self.l3 * self.l3) / (2.0 * self.l2 * ac)
        cos_bac = max(-1.0, min(1.0, cos_bac))
        bac = math.acos(cos_bac)

        cos_caf = max(-1.0, min(1.0, af / ac))
        caf = math.acos(cos_caf)
        zf_flag = 1.0 if cf >= 0 else -1.0
        theta_shoulder = math.degrees(caf * zf_flag + bac)

        # 7. Wrist Pitch (theta_wrist_pitch)
        theta_wrist_pitch = alpha_pitch - theta_shoulder + theta_elbow

        # 8. Map to Hardware Pulses (Tankbot IDs 1 to 6)
        pulses = self._angles_to_pulses(
            theta_base=theta_base,
            theta_shoulder=theta_shoulder,
            theta_elbow=theta_elbow,
            theta_wrist_pitch=theta_wrist_pitch,
            wrist_roll_pulse=wrist_roll_pulse,
            claw_pulse=claw_pulse
        )

        if pulses is None:
            return None

        return KinematicSolution(
            reachable=True,
            coordinate=(x, y, z),
            alpha_pitch=alpha_pitch,
            theta_base=round(theta_base, 1),
            theta_shoulder=round(theta_shoulder, 1),
            theta_elbow=round(theta_elbow, 1),
            theta_wrist_pitch=round(theta_wrist_pitch, 1),
            theta_wrist_roll=round((wrist_roll_pulse - 500) * 0.24, 1),
            pulses=pulses
        )

    def solve_pitch_search(
        self,
        target_xyz: Tuple[float, float, float],
        alpha_target: float = -90.0,
        alpha_min: float = -90.0,
        alpha_max: float = 0.0,
        step: float = 2.0,
        wrist_roll_pulse: int = 500,
        claw_pulse: int = 500
    ) -> Optional[KinematicSolution]:
        """
        Searches across pitch range to find the most natural reachable pose.
        """
        # Try requested angle first
        sol = self.solve_exact(target_xyz, alpha_target, wrist_roll_pulse, claw_pulse)
        if sol is not None:
            return sol

        best_sol = None
        min_delta = 999.0

        for a in np.arange(alpha_min, alpha_max + step, step):
            s = self.solve_exact(target_xyz, float(a), wrist_roll_pulse, claw_pulse)
            if s is not None:
                delta = abs(float(a) - alpha_target)
                if delta < min_delta:
                    min_delta = delta
                    best_sol = s

        return best_sol

    def solve(
        self,
        target_xyz: Tuple[float, float, float],
        alpha_pitch: float = -90.0,
        wrist_roll_pulse: int = 500,
        claw_pulse: int = 500
    ) -> Optional[KinematicSolution]:
        """
        Solves IK for target (X, Y, Z) in cm.
        Attempts requested alpha_pitch first; if unreachable at that exact angle,
        automatically searches across pitch envelope to find a reachable solution.
        """
        return self.solve_pitch_search(
            target_xyz=target_xyz,
            alpha_target=alpha_pitch,
            wrist_roll_pulse=wrist_roll_pulse,
            claw_pulse=claw_pulse
        )

    def _angles_to_pulses(
        self,
        theta_base: float,
        theta_shoulder: float,
        theta_elbow: float,
        theta_wrist_pitch: float,
        wrist_roll_pulse: int,
        claw_pulse: int
    ) -> Optional[Dict[int, int]]:
        """
        Converts geometric angles to Tankbot 0-1000 pulse commands:
        ID 1: Base (center 500 = 90 deg forward)
        ID 2: Shoulder (center 500 = 90 deg vertical)
        ID 3: Elbow (center 500 = 90 deg bend)
        ID 4: Wrist Pitch (center 500 = straight)
        ID 5: Wrist Roll (clamped pulse)
        ID 6: Claw (clamped pulse)
        """
        try:
            pulse_base = int(round(500 + (theta_base - 90.0) * self.PULSE_PER_DEG))
            pulse_shoulder = int(round(500 - (90.0 - theta_shoulder) * self.PULSE_PER_DEG))
            pulse_elbow = int(round(500 + (theta_elbow - 90.0) * self.PULSE_PER_DEG))
            pulse_wrist = int(round(500 + theta_wrist_pitch * self.PULSE_PER_DEG))

            def clamp(val, low=50, high=950):
                return max(low, min(high, val))

            # Hardware limits validation
            if not (50 <= pulse_base <= 950 and 50 <= pulse_shoulder <= 950 and 50 <= pulse_elbow <= 950 and 50 <= pulse_wrist <= 950):
                return None

            return {
                1: clamp(pulse_base),
                2: clamp(pulse_shoulder),
                3: clamp(pulse_elbow),
                4: clamp(pulse_wrist),
                5: max(0, min(1000, int(wrist_roll_pulse))),
                6: max(130, min(875, int(claw_pulse)))
            }
        except Exception as e:
            logger.debug(f"Pulse conversion error: {e}")
            return None
