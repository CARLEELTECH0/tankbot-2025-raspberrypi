"""
Tankbot Autonomous "See-Think-Act" Perception-Action Engine
Multi-threaded closed-loop autonomous system:
- Thread 1 (Perception): Captures 640x480 @ 30 FPS, detects target, calculates center error (ex, ey)
- Thread 2 (Cognition & IK): State machine (SEARCHING, TRACKING, LOCKING, GRASPING, DEPOSITING), solves 6-DOF IK
- Thread 3 (Action): Streams computed joint angles and chassis motor speeds to HAL layer without latency
"""

import time
import queue
import logging
import threading
from enum import Enum
from typing import Optional, Dict, Tuple
import numpy as np

from vision.detector import ObjectDetector, VisionTarget
from vision.camera_processor import CameraProcessor
from kinematics.ik_solver import IKSolver, KinematicSolution
from hal.motor_chassis import MotorChassis
from hal.arm_servos import ArmServos

logger = logging.getLogger("Tankbot.Core.Engine")


class AutonomousState(Enum):
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    TRACKING = "TRACKING"
    LOCKING = "LOCKING"
    GRASPING = "GRASPING"
    DEPOSITING = "DEPOSITING"
    RETURNING = "RETURNING"


class AutonomousEngine:
    """
    Closed-loop See-Think-Act Engine running 3 asynchronous daemon threads.
    Thread-safe synchronization allows continuous physical motion while perception processes frames.
    """

    def __init__(
        self,
        camera: Optional[CameraProcessor] = None,
        detector: Optional[ObjectDetector] = None,
        ik_solver: Optional[IKSolver] = None,
        chassis: Optional[MotorChassis] = None,
        servos: Optional[ArmServos] = None
    ):
        # 1. Hardware & Perception Subsystems
        self.camera = camera or CameraProcessor()
        self.detector = detector or ObjectDetector()
        self.ik_solver = ik_solver or IKSolver()
        self.chassis = chassis or MotorChassis()
        self.servos = servos or ArmServos()

        # 2. State Machine Variables
        self.state = AutonomousState.IDLE
        self.mode = "MANUAL"  # "MANUAL", "AUTO_TRACK", "AUTO_PICK_PLACE"
        self.target_color = "red"

        # 3. Thread Synchronization Locks & Shared State
        self._lock_perception = threading.Lock()
        self._current_target: VisionTarget = VisionTarget(status="IDLE")
        self._annotated_frame: Optional[np.ndarray] = None

        self._lock_action = threading.Lock()
        self._target_servo_pulses: Optional[Dict[int, int]] = None
        self._target_motor_speeds: Tuple[int, int] = (0, 0)
        self._servo_move_time: int = 50

        # State tracking timestamps
        self._state_start_time = time.time()
        self._lock_acquired_time = 0.0
        self._grasp_step = 0
        self._grasp_step_time = 0.0

        # Deposit Bin Locations (X, Y, Z) in cm
        self.bins = {
            "red":   (-14.0, 12.0, 3.0),
            "green": (-14.0, 6.0,  3.0),
            "blue":  (-14.0, 0.0,  3.0),
        }

        # Thread controls
        self._running = False
        self._thread_perception: Optional[threading.Thread] = None
        self._thread_cognition: Optional[threading.Thread] = None
        self._thread_action: Optional[threading.Thread] = None

        logger.info("AutonomousEngine initialized (Perception, Cognition, Action).")

    def start(self):
        """Launches the 3 asynchronous worker threads"""
        if self._running:
            return

        self._running = True
        self.camera.start()

        # Thread 1: Perception
        self._thread_perception = threading.Thread(
            target=self._perception_thread_loop,
            name="Thread-1-Perception",
            daemon=True
        )

        # Thread 2: Cognition & IK
        self._thread_cognition = threading.Thread(
            target=self._cognition_thread_loop,
            name="Thread-2-Cognition",
            daemon=True
        )

        # Thread 3: Action Execution
        self._thread_action = threading.Thread(
            target=self._action_thread_loop,
            name="Thread-3-Action",
            daemon=True
        )

        self._thread_perception.start()
        self._thread_cognition.start()
        self._thread_action.start()
        logger.info("All 3 See-Think-Act threads launched successfully.")

    def stop(self):
        """Stops all threads and halts motors"""
        self._running = False
        self.chassis.stop()
        self.camera.stop()
        logger.info("AutonomousEngine stopped.")

    def set_target_color(self, color: str):
        """Configures target color to track ('red', 'green', 'blue', 'none')"""
        self.target_color = color.lower()
        self.detector.set_target_color(color)
        if hasattr(self.camera, "set_simulation_color"):
            self.camera.set_simulation_color(color)
        logger.info(f"Engine target color set to: {self.target_color}")

    def set_mode(self, mode: str):
        """Switches engine mode: 'MANUAL', 'AUTO_TRACK', 'AUTO_PICK_PLACE'"""
        self.mode = mode.upper()
        if self.mode == "MANUAL":
            self.state = AutonomousState.IDLE
            self.chassis.stop()
        elif self.mode in ["AUTO_TRACK", "AUTO_PICK_PLACE"]:
            self.state = AutonomousState.SEARCHING
            self._state_start_time = time.time()
        logger.info(f"Engine mode switched to: {self.mode}")

    def trigger_pick_and_place(self):
        """Forces start of autonomous pick and place routine"""
        self.mode = "AUTO_PICK_PLACE"
        self.state = AutonomousState.TRACKING
        self._grasp_step = 0
        self._grasp_step_time = time.time()

    # =========================================================================
    # THREAD 1: PERCEPTION (OpenCV Video Acquisition & Error Extraction)
    # =========================================================================
    def _perception_thread_loop(self):
        """
        Continuously reads 640x480 @ 30 FPS video frames from USB-A camera,
        applies LAB/HSV thresholding, calculates bounding boxes and optical axis error (ex, ey).
        """
        while self._running:
            try:
                frame = self.camera.get_frame()
                if frame is not None:
                    annotated, target = self.detector.detect(frame, annotate=True)
                    with self._lock_perception:
                        self._current_target = target
                        self._annotated_frame = annotated
                time.sleep(0.01)  # ~30 FPS poll
            except Exception as e:
                logger.debug(f"Perception thread error: {e}")
                time.sleep(0.05)

    # =========================================================================
    # THREAD 2: COGNITION & INVERSE KINEMATICS (State Machine & Decision Making)
    # =========================================================================
    def _cognition_thread_loop(self):
        """
        Evaluates state machine and computes IK target joint angles.
        Prevents camera lag from interfering with decision making.
        """
        while self._running:
            try:
                now = time.time()

                # Read latest perception snapshot
                with self._lock_perception:
                    target = self._current_target

                if self.mode == "MANUAL":
                    self.state = AutonomousState.IDLE
                    time.sleep(0.05)
                    continue

                # State Machine
                if self.state == AutonomousState.SEARCHING:
                    if target.detected:
                        self.state = AutonomousState.TRACKING
                        self._state_start_time = now
                        logger.info(f"Target '{target.color}' acquired. Switching to TRACKING.")
                    else:
                        # Slow rotational chassis search or arm search
                        with self._lock_action:
                            self._target_motor_speeds = (0, 0)
                            # Gently sweep arm base
                            sweep_pulse = int(500 + 150 * np.sin((now - self._state_start_time) * 1.5))
                            self._target_servo_pulses = {1: sweep_pulse, 2: 500, 3: 350, 4: 100, 5: 500, 6: 200}
                            self._servo_move_time = 100

                elif self.state == AutonomousState.TRACKING:
                    if not target.detected:
                        # Target lost; return to searching after 1.5s
                        if now - self._state_start_time > 1.5:
                            self.state = AutonomousState.SEARCHING
                            self._state_start_time = now
                    else:
                        self._state_start_time = now

                        # Solve IK for arm to track target in 3D ground space
                        sol = self.ik_solver.solve_pitch_search(
                            target_xyz=(target.world_x, target.world_y, 6.0),  # hover 6cm above target
                            alpha_target=-70.0,
                            wrist_roll_pulse=target.wrist_pulse,
                            claw_pulse=200  # claw open while tracking
                        )

                        if sol and sol.reachable:
                            with self._lock_action:
                                self._target_servo_pulses = sol.pulses
                                self._servo_move_time = 60

                        # Chassis tracking: steer robot to center target horizontally
                        steer_speed = 0
                        if abs(target.error_norm_x) > 0.15:
                            steer_speed = int(target.error_norm_x * 40.0)
                        with self._lock_action:
                            self._target_motor_speeds = (steer_speed, -steer_speed)

                        # Check if target is centered & stable for locking
                        if abs(target.error_norm_x) < 0.15 and abs(target.error_norm_y) < 0.15:
                            if self._lock_acquired_time == 0.0:
                                self._lock_acquired_time = now
                            elif now - self._lock_acquired_time > 1.0:
                                self.state = AutonomousState.LOCKING
                                logger.info("Target locked! Arm & Chassis centered.")
                        else:
                            self._lock_acquired_time = 0.0

                elif self.state == AutonomousState.LOCKING:
                    # Stop chassis and prepare for pick
                    with self._lock_action:
                        self._target_motor_speeds = (0, 0)

                    if self.mode == "AUTO_PICK_PLACE":
                        self.state = AutonomousState.GRASPING
                        self._grasp_step = 1
                        self._grasp_step_time = now
                        logger.info("Executing Autonomous GRASPING routine...")
                    else:
                        # Auto track mode remains in locked hover
                        self.state = AutonomousState.TRACKING

                elif self.state == AutonomousState.GRASPING:
                    self._handle_grasping_sequence(now, target)

                elif self.state == AutonomousState.DEPOSITING:
                    self._handle_depositing_sequence(now, target)

                elif self.state == AutonomousState.RETURNING:
                    if now - self._state_start_time > 1.5:
                        self.servos.home(1000)
                        self.state = AutonomousState.SEARCHING
                        self._state_start_time = now
                        logger.info("Pick & Place cycle complete. Ready for next target.")

                time.sleep(0.03)  # ~33 Hz cognition update rate

            except Exception as e:
                logger.error(f"Cognition thread error: {e}", exc_info=True)
                time.sleep(0.1)

    def _handle_grasping_sequence(self, now: float, target: VisionTarget):
        """Sequences arm grasp stages cleanly without thread blocking"""
        step_elapsed = now - self._grasp_step_time
        wx, wy = target.world_x, target.world_y
        wrist_p = target.wrist_pulse

        if self._grasp_step == 1:
            # Step 1: Hover above target with open claw
            sol = self.ik_solver.solve_pitch_search((wx, wy, 8.0), -80.0, wrist_p, 200)
            if sol:
                with self._lock_action:
                    self._target_servo_pulses = sol.pulses
                    self._servo_move_time = 800
            if step_elapsed > 0.9:
                self._grasp_step = 2
                self._grasp_step_time = now

        elif self._grasp_step == 2:
            # Step 2: Descend to table height
            sol = self.ik_solver.solve_pitch_search((wx, wy, 2.0), -85.0, wrist_p, 200)
            if sol:
                with self._lock_action:
                    self._target_servo_pulses = sol.pulses
                    self._servo_move_time = 700
            if step_elapsed > 0.8:
                self._grasp_step = 3
                self._grasp_step_time = now

        elif self._grasp_step == 3:
            # Step 3: Close claw to grip object
            with self._lock_action:
                if self._target_servo_pulses:
                    self._target_servo_pulses[6] = 550  # close claw
                    self._servo_move_time = 500
            if step_elapsed > 0.7:
                self._grasp_step = 4
                self._grasp_step_time = now

        elif self._grasp_step == 4:
            # Step 4: Lift upward
            sol = self.ik_solver.solve_pitch_search((wx, wy, 12.0), -70.0, wrist_p, 550)
            if sol:
                with self._lock_action:
                    self._target_servo_pulses = sol.pulses
                    self._servo_move_time = 800
            if step_elapsed > 0.9:
                self.state = AutonomousState.DEPOSITING
                self._grasp_step = 1
                self._grasp_step_time = now
                logger.info(f"Block grasped. Moving to deposit bin.")

    def _handle_depositing_sequence(self, now: float, target: VisionTarget):
        """Deposits block into corresponding color container"""
        step_elapsed = now - self._grasp_step_time
        bin_xyz = self.bins.get(target.color, (-14.0, 6.0, 3.0))

        if self._grasp_step == 1:
            # Step 1: Move above bin
            sol = self.ik_solver.solve_pitch_search((bin_xyz[0], bin_xyz[1], bin_xyz[2] + 7.0), -70.0, 500, 550)
            if sol:
                with self._lock_action:
                    self._target_servo_pulses = sol.pulses
                    self._servo_move_time = 1200
            if step_elapsed > 1.3:
                self._grasp_step = 2
                self._grasp_step_time = now

        elif self._grasp_step == 2:
            # Step 2: Lower to bin
            sol = self.ik_solver.solve_pitch_search((bin_xyz[0], bin_xyz[1], bin_xyz[2]), -80.0, 500, 550)
            if sol:
                with self._lock_action:
                    self._target_servo_pulses = sol.pulses
                    self._servo_move_time = 600
            if step_elapsed > 0.7:
                self._grasp_step = 3
                self._grasp_step_time = now

        elif self._grasp_step == 3:
            # Step 3: Open claw to release
            with self._lock_action:
                if self._target_servo_pulses:
                    self._target_servo_pulses[6] = 200  # open claw
                    self._servo_move_time = 400
            if step_elapsed > 0.6:
                self.state = AutonomousState.RETURNING
                self._state_start_time = now
                self.servos.home(1000)

    # =========================================================================
    # THREAD 3: ACTION EXECUTION (Hardware Command Streaming)
    # =========================================================================
    def _action_thread_loop(self):
        """
        Continuously streams computed servo joint pulses and motor speeds to the custom HAL layer.
        Runs at 50 Hz for ultra-smooth physical actuation.
        """
        while self._running:
            try:
                with self._lock_action:
                    pulses = self._target_servo_pulses
                    move_time = self._servo_move_time
                    m_left, m_right = self._target_motor_speeds

                # Stream to hardware
                if pulses:
                    self.servos.set_multiple_servos(pulses, move_time)

                if self.mode != "MANUAL":
                    self.chassis.set_motors(m_left, m_right)

                time.sleep(0.02)  # 50 Hz control rate

            except Exception as e:
                logger.debug(f"Action thread error: {e}")
                time.sleep(0.05)

    def get_telemetry_snapshot(self) -> dict:
        """Returns thread-safe telemetry snapshot for Mobile UI streaming"""
        with self._lock_perception:
            target = self._current_target

        return {
            "engine_state": self.state.value,
            "engine_mode": self.mode,
            "target_color": self.target_color,
            "detected": target.detected,
            "error_x": target.error_x,
            "error_y": target.error_y,
            "world_coords": {
                "x": target.world_x,
                "y": target.world_y,
                "z": target.world_z
            },
            "rotation_angle": target.rotation_angle,
            "wrist_pulse": target.wrist_pulse,
            "camera_fps": self.camera.get_fps(),
        }

    def get_annotated_jpeg(self) -> Optional[bytes]:
        """Returns latest perception annotated frame encoded as JPEG"""
        with self._lock_perception:
            frame = self._annotated_frame

        if frame is None:
            return self.camera.get_jpeg()

        try:
            import cv2
            ret, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ret:
                return buf.tobytes()
        except Exception:
            pass

        return None
