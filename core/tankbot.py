"""
Tankbot Core Controller
Coordinates motors, 6-DOF arm, sensors, autonomous routines, and safety watchdogs.
"""

import time
import asyncio
import logging
from typing import Optional, Dict
from config import ULTRASONIC_CONFIG, SAFETY_CONFIG, CAMERA_CONFIG, VISION_CONFIG
from hardware import (
    HAL,
    MotorController,
    ServoController,
    UltrasonicSensor,
    LineFollower,
    IMUSensor
)
from vision import CameraProcessor, ColorTracker, IKBridge
from core.telemetry import TelemetryData

logger = logging.getLogger("Tankbot.Core")


class Tankbot:
    """Master Robot Controller"""

    def __init__(self):
        self.hal = HAL()
        self.motors = MotorController()
        self.servos = ServoController()
        self.ultrasonic = UltrasonicSensor()
        self.line_follower = LineFollower()
        self.imu = IMUSensor()

        # Computer Vision & Inverse Kinematics Stack (v2.0)
        self.camera = CameraProcessor(
            device=CAMERA_CONFIG.get("device", "/dev/video0"),
            resolution=CAMERA_CONFIG.get("resolution", (640, 480)),
            target_fps=CAMERA_CONFIG.get("fps", 30),
            fourcc=CAMERA_CONFIG.get("fourcc", "MJPG")
        )
        self.color_tracker = ColorTracker(
            target_color=VISION_CONFIG.get("default_target_color", "red"),
            color_space=VISION_CONFIG.get("color_space", "LAB"),
            map_param=VISION_CONFIG.get("map_param", 0.05),
            image_center_distance=VISION_CONFIG.get("image_center_distance", 20.0),
            min_contour_area=VISION_CONFIG.get("min_contour_area", 300.0),
            trigger_contour_area=VISION_CONFIG.get("trigger_contour_area", 1800.0)
        )
        self.ik_bridge = IKBridge()
        self.camera.start()

        self.mode = "MANUAL"  # MANUAL, OBSTACLE_AVOIDANCE, OBJECT_FOLLOW, LINE_FOLLOW, VISION_TRACK, VISION_PICK_PLACE
        self.emergency_stopped = False
        self.active_sequence_name: Optional[str] = None
        self._sequence_task: Optional[asyncio.Task] = None

        self.last_heartbeat = time.time()
        self.heartbeat_timeout = SAFETY_CONFIG.get("heartbeat_timeout_s", 2.5)

        # Autonomous loop state
        self._autonomous_task: Optional[asyncio.Task] = None
        self._running = False
        
        # Initialize arm to home
        self.servos.home()

        self.telemetry = TelemetryData(
            is_simulation=self.hal.is_simulation,
            servo_positions=self.servos.get_positions()
        )
        logger.info("Tankbot core system initialized with CV & IK v2.0.")

    async def start(self):
        """Starts background autonomous loop and safety monitor"""
        self._running = True
        self.camera.start()
        self._autonomous_task = asyncio.create_task(self._control_loop())
        logger.info("Tankbot background tasks started.")

    async def stop(self):
        """Graceful shutdown"""
        self._running = False
        if self._autonomous_task:
            self._autonomous_task.cancel()
        if self._sequence_task:
            self._sequence_task.cancel()
        self.camera.stop()
        self.motors.stop()
        self.hal.cleanup()
        logger.info("Tankbot stopped cleanly.")

    def heartbeat(self):
        """Refreshes connection watchdog from mobile client"""
        self.last_heartbeat = time.time()

    def set_mode(self, mode: str):
        """Switches operating mode"""
        valid_modes = [
            "MANUAL",
            "OBSTACLE_AVOIDANCE",
            "OBJECT_FOLLOW",
            "LINE_FOLLOW",
            "VISION_TRACK",
            "VISION_PICK_PLACE"
        ]
        if mode in valid_modes:
            if self._sequence_task and not self._sequence_task.done():
                self._sequence_task.cancel()
            self.mode = mode
            self.motors.stop()
            self._sync_telemetry_state()
            logger.info(f"Tankbot mode switched to: {mode}")

    def emergency_stop(self):
        """Immediately halts all physical motion"""
        self.emergency_stopped = True
        self.motors.stop()
        if self._sequence_task and not self._sequence_task.done():
            self._sequence_task.cancel()
        self.mode = "MANUAL"
        self._sync_telemetry_state()
        logger.warning("EMERGENCY STOP TRIGGERED!")

    def reset_emergency_stop(self):
        """Clears emergency state"""
        self.emergency_stopped = False
        self.motors.stop()
        self._sync_telemetry_state()
        logger.info("Emergency stop cleared.")

    def drive_joystick(self, throttle: float, steering: float):
        """Manual joystick drive"""
        if self.emergency_stopped or self.mode != "MANUAL":
            return
        self.motors.drive_joystick(throttle, steering)
        self._sync_telemetry_state()

    def drive_direct(self, left_speed: int, right_speed: int):
        """Direct track drive"""
        if self.emergency_stopped or self.mode != "MANUAL":
            return
        self.motors.set_motors(left_speed, right_speed)
        self._sync_telemetry_state()

    def set_servo(self, servo_id: int, position: int, duration_ms: int = 50):
        """Adjusts a single arm joint"""
        if self.emergency_stopped:
            return
        self.servos.set_servo(servo_id, position, duration_ms)
        self.telemetry.servo_positions = self.servos.get_positions()

    def home_arm(self):
        """Returns arm to safe home position"""
        if self.emergency_stopped:
            return
        self.servos.home()
        self.telemetry.servo_positions = self.servos.get_positions()

    def trigger_preset(self, preset_name: str):
        """Triggers a pre-programmed arm action"""
        if self.emergency_stopped:
            return

        if preset_name == "grab":
            if self._sequence_task and not self._sequence_task.done():
                self._sequence_task.cancel()
            self.active_sequence_name = "grab"
            self.telemetry.active_sequence = "grab"
            self._sequence_task = asyncio.create_task(self._run_grab_sequence())
        elif preset_name == "home":
            self.home_arm()
        elif preset_name == "open_claw":
            self.servos.open_claw()
            self.telemetry.servo_positions = self.servos.get_positions()
        elif preset_name == "close_claw":
            self.servos.close_claw()
            self.telemetry.servo_positions = self.servos.get_positions()
        elif preset_name == "rest":
            self.servos.set_multiple({1: 500, 2: 200, 3: 150, 4: 100, 5: 500, 6: 200}, 1000)
            self.telemetry.servo_positions = self.servos.get_positions()
        elif preset_name == "vision_pick_and_place":
            if self._sequence_task and not self._sequence_task.done():
                self._sequence_task.cancel()
            self.active_sequence_name = "vision_pick_and_place"
            self.telemetry.active_sequence = "vision_pick_and_place"
            self._sequence_task = asyncio.create_task(self._run_vision_pick_and_place())
        elif preset_name == "wave":
            self.servos.set_multiple({1: 500, 2: 750, 3: 500, 4: 300, 5: 700, 6: 500}, 800)
            self.telemetry.servo_positions = self.servos.get_positions()

    async def _run_grab_sequence(self):
        try:
            await self.servos.execute_grab_sequence()
        except asyncio.CancelledError:
            pass
        finally:
            self.active_sequence_name = None
            self.telemetry.active_sequence = None

    async def _run_vision_pick_and_place(self):
        """
        Autonomous 8-step visual pick and place routine.
        Uses detected target coordinates (world_x, world_y) from CV
        and computes inverse kinematics to grasp and sort colored block.
        """
        try:
            logger.info("Starting visual pick-and-place routine...")
            det = self.color_tracker.last_result
            if not det.detected:
                logger.warning("No target detected for vision pick & place! Aborting.")
                return

            wx, wy = det.world_x, det.world_y
            wrist_pulse = det.wrist_servo_pulse
            color = det.color

            # Step 1: Open claw and hover over target
            self.servos.open_claw()
            hover_pose = self.ik_bridge.calculate_target_pose(
                wx, wy, world_z=8.0, alpha=-80.0, wrist_roll_pulse=wrist_pulse, claw_pulse=200
            )
            if hover_pose:
                self.servos.set_multiple(hover_pose, duration_ms=1000)
                await asyncio.sleep(1.2)

            # Step 2: Descend to grasping elevation
            grasp_pose = self.ik_bridge.calculate_target_pose(
                wx, wy, world_z=2.0, alpha=-85.0, wrist_roll_pulse=wrist_pulse, claw_pulse=200
            )
            if grasp_pose:
                self.servos.set_multiple(grasp_pose, duration_ms=800)
                await asyncio.sleep(1.0)

            # Step 3: Close claw to grasp block
            self.servos.close_claw(duration_ms=600)
            await asyncio.sleep(0.8)

            # Step 4: Lift object upward
            lift_pose = self.ik_bridge.calculate_target_pose(
                wx, wy, world_z=12.0, alpha=-70.0, wrist_roll_pulse=wrist_pulse, claw_pulse=550
            )
            if lift_pose:
                self.servos.set_multiple(lift_pose, duration_ms=800)
                await asyncio.sleep(1.0)

            # Step 5: Rotate and move to sorted color bin
            bin_coords = self.ik_bridge.get_bin_coordinate(color)
            bin_pose = self.ik_bridge.calculate_target_pose(
                bin_coords[0], bin_coords[1], world_z=bin_coords[2] + 6.0, alpha=-70.0, claw_pulse=550
            )
            if bin_pose:
                self.servos.set_multiple(bin_pose, duration_ms=1200)
                await asyncio.sleep(1.4)

            # Step 6: Lower to bin
            bin_drop_pose = self.ik_bridge.calculate_target_pose(
                bin_coords[0], bin_coords[1], world_z=bin_coords[2], alpha=-80.0, claw_pulse=550
            )
            if bin_drop_pose:
                self.servos.set_multiple(bin_drop_pose, duration_ms=600)
                await asyncio.sleep(0.8)

            # Step 7: Open claw to release
            self.servos.open_claw(duration_ms=500)
            await asyncio.sleep(0.7)

            # Step 8: Return home
            self.servos.home(duration_ms=1200)
            await asyncio.sleep(1.3)
            logger.info(f"Visual pick-and-place complete for {color} block!")

        except asyncio.CancelledError:
            logger.info("Visual pick-and-place cancelled.")
        finally:
            self.active_sequence_name = None
            self.telemetry.active_sequence = None

    def _sync_telemetry_state(self):
        """Instantly updates telemetry fields without waiting for control loop tick"""
        self.telemetry.emergency_stopped = self.emergency_stopped
        self.telemetry.mode = self.mode
        self.telemetry.left_speed = self.motors.current_left_speed
        self.telemetry.right_speed = self.motors.current_right_speed
        self.telemetry.is_moving = (self.motors.current_left_speed != 0 or self.motors.current_right_speed != 0)
        self.telemetry.servo_positions = self.servos.get_positions()
        self.telemetry.active_sequence = self.active_sequence_name

    async def _control_loop(self):
        """Continuous background loop for autonomous navigation & telemetry updates"""
        avoid_turn_until = 0.0

        while self._running:
            try:
                now = time.time()
                is_heartbeat_ok = (now - self.last_heartbeat) < self.heartbeat_timeout

                # Safety watchdog: stop if disconnected in manual mode
                if not is_heartbeat_ok and self.mode == "MANUAL" and not self.emergency_stopped:
                    if self.motors.current_left_speed != 0 or self.motors.current_right_speed != 0:
                        self.motors.stop()

                # Ultrasonic measurement
                sim_speed = (self.motors.current_left_speed + self.motors.current_right_speed) // 2
                dist_mm = self.ultrasonic.measure_distance_mm(sim_speed)
                dist_cm = round(dist_mm / 10.0, 1)

                # IMU & Line reading
                imu_data = self.imu.read_posture()
                line_states = self.line_follower.read_sensors()

                # Computer Vision Processing
                frame = self.camera.get_frame()
                if frame is not None and self.color_tracker.target_color not in ["none", "off"]:
                    _, det = self.color_tracker.process_frame(frame, annotate=True)
                    self.telemetry.vision_detected = det.detected
                    self.telemetry.vision_target_color = det.color
                    self.telemetry.vision_coords = {"x": det.world_x, "y": det.world_y, "z": det.world_z}
                    self.telemetry.vision_rotation_angle = det.rotation_angle
                    self.telemetry.vision_wrist_pulse = det.wrist_servo_pulse
                    self.telemetry.vision_status = det.status
                    self.telemetry.camera_fps = self.camera.get_fps()

                    # Autonomous Vision Tracking
                    if not self.emergency_stopped and self.mode == "VISION_TRACK" and det.detected:
                        track_pose = self.ik_bridge.calculate_target_pose(
                            det.world_x, det.world_y, world_z=6.0, alpha=-70.0,
                            wrist_roll_pulse=det.wrist_servo_pulse, claw_pulse=200
                        )
                        if track_pose:
                            self.servos.set_multiple(track_pose, duration_ms=60)

                # Autonomous Mode Execution
                if not self.emergency_stopped:
                    if self.mode == "OBSTACLE_AVOIDANCE":
                        # Logic ported from STM32 Lesson 2 (STM32_Barrier)
                        if now < avoid_turn_until:
                            pass
                        elif dist_mm > ULTRASONIC_CONFIG["avoidance_threshold_mm"]:
                            self.motors.set_motors(80, 80)
                        else:
                            self.motors.set_motors(80, -80)
                            avoid_turn_until = now + 1.0

                    elif self.mode == "OBJECT_FOLLOW":
                        # Logic ported from STM32 Lesson 1 (STM32_Follow)
                        if 20 < dist_mm < ULTRASONIC_CONFIG["follow_min_mm"]:
                            self.motors.set_motors(-70, -70)
                        elif ULTRASONIC_CONFIG["follow_max_mm"] < dist_mm < ULTRASONIC_CONFIG["follow_detect_mm"]:
                            self.motors.set_motors(70, 70)
                        else:
                            self.motors.set_motors(0, 0)

                    elif self.mode == "LINE_FOLLOW":
                        # Autonomous line following ported from STM32 Lesson 4 (I2C 4-Channel Line Tracker)
                        left_spd, right_spd = self.line_follower.get_motor_speeds(base_speed=80)
                        self.motors.set_motors(left_spd, right_spd)

                # Update Telemetry Snapshot
                self.telemetry.timestamp = now
                self.telemetry.distance_mm = dist_mm
                self.telemetry.distance_cm = dist_cm
                self.telemetry.line_sensors = line_states
                self.telemetry.pitch = imu_data.get("pitch", 0.0)
                self.telemetry.roll = imu_data.get("roll", 0.0)
                self.telemetry.is_level = imu_data.get("is_level", True)
                self.telemetry.heartbeat_ok = is_heartbeat_ok
                self._sync_telemetry_state()

                await asyncio.sleep(0.05)  # 20 Hz loop rate (50ms)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in Tankbot control loop: {e}", exc_info=True)
                await asyncio.sleep(0.1)

    def get_telemetry(self) -> dict:
        self._sync_telemetry_state()
        return self.telemetry.to_dict()
