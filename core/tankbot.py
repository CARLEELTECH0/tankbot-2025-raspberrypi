"""
Tankbot Core Controller
Coordinates motors, 6-DOF arm, sensors, autonomous routines, and safety watchdogs.
"""

import time
import asyncio
import logging
from typing import Optional, Dict
from config import ULTRASONIC_CONFIG, SAFETY_CONFIG
from hardware import (
    HAL,
    MotorController,
    ServoController,
    UltrasonicSensor,
    LineFollower,
    IMUSensor
)
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

        self.mode = "MANUAL"  # MANUAL, OBSTACLE_AVOIDANCE, OBJECT_FOLLOW, LINE_FOLLOW
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
        logger.info("Tankbot core system initialized.")

    async def start(self):
        """Starts background autonomous loop and safety monitor"""
        self._running = True
        self._autonomous_task = asyncio.create_task(self._control_loop())
        logger.info("Tankbot background tasks started.")

    async def stop(self):
        """Graceful shutdown"""
        self._running = False
        if self._autonomous_task:
            self._autonomous_task.cancel()
        if self._sequence_task:
            self._sequence_task.cancel()
        self.motors.stop()
        self.hal.cleanup()
        logger.info("Tankbot stopped cleanly.")

    def heartbeat(self):
        """Refreshes connection watchdog from mobile client"""
        self.last_heartbeat = time.time()

    def set_mode(self, mode: str):
        """Switches operating mode"""
        valid_modes = ["MANUAL", "OBSTACLE_AVOIDANCE", "OBJECT_FOLLOW", "LINE_FOLLOW"]
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

                # IMU reading
                imu_data = self.imu.read_posture()
                line_states = self.line_follower.read_sensors()

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
                        # Logic ported from STM32 linefollow()
                        steering = self.line_follower.get_steering_recommendation()
                        if steering < 0:
                            self.motors.set_motors(60, 85)
                        elif steering > 0:
                            self.motors.set_motors(85, 60)
                        else:
                            self.motors.set_motors(75, 75)

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
