"""
Tankbot Chassis Motor Controller - Hardware Abstraction Layer
Ported from STM32 Motor.c / Motor.h
Directly drives IN1/IN2 (Left Track M1) and IN3/IN4 (Right Track M2) as independent PWM channels.
No enable pins required (matches Hiwonder OpenCar4in1 baseboard L298P with Enable pins pulled HIGH).
"""

import time
import logging
import threading
from typing import Dict, Tuple, Optional
from config import MOTOR_CONFIG
from hardware.hal import HAL

logger = logging.getLogger("Tankbot.HAL.Chassis")


class MotorChassis:
    """
    Hardware Abstraction for Tankbot Tracked Differential Drive Chassis.
    Translates high-level steering & speed commands into 4-channel PWM signals.
    """

    def __init__(self, config: Optional[dict] = None):
        self.config = config or MOTOR_CONFIG
        self.hal = HAL()

        self.in1_pin = self.config.get("in1", 20)  # Left M1 Forward
        self.in2_pin = self.config.get("in2", 21)  # Left M1 Reverse
        self.in3_pin = self.config.get("in3", 26)  # Right M2 Forward
        self.in4_pin = self.config.get("in4", 16)  # Right M2 Reverse

        self.invert_left = self.config.get("invert_left", False)
        self.invert_right = self.config.get("invert_right", False)
        self.deadzone = self.config.get("deadzone", 5)
        self.pwm_freq = self.config.get("pwm_freq", 1000)

        # Thread safety lock
        self._lock = threading.Lock()

        # Dedicated 1 kHz PWM outputs
        self.pwm_in1 = self.hal.get_pwm(self.in1_pin, self.pwm_freq)
        self.pwm_in2 = self.hal.get_pwm(self.in2_pin, self.pwm_freq)
        self.pwm_in3 = self.hal.get_pwm(self.in3_pin, self.pwm_freq)
        self.pwm_in4 = self.hal.get_pwm(self.in4_pin, self.pwm_freq)

        self.pwm_in1.start(0)
        self.pwm_in2.start(0)
        self.pwm_in3.start(0)
        self.pwm_in4.start(0)

        self.left_speed = 0
        self.right_speed = 0
        self.stop()
        logger.info("MotorChassis initialized with 4-channel PWM (Zero Enable Pins).")

    def set_motors(self, left: int, right: int):
        """
        Direct Left/Right track speed (-100 to +100).
        Ported from STM32 MotorControl(int8 m1Speed, int8 m2Speed).
        """
        with self._lock:
            left_clamped = max(-100, min(100, int(left)))
            right_clamped = max(-100, min(100, int(right)))

            if abs(left_clamped) < self.deadzone:
                left_clamped = 0
            if abs(right_clamped) < self.deadzone:
                right_clamped = 0

            if self.invert_left:
                left_clamped = -left_clamped
            if self.invert_right:
                right_clamped = -right_clamped

            self.left_speed = left_clamped
            self.right_speed = right_clamped

            self._apply_left(left_clamped)
            self._apply_right(right_clamped)

    def _apply_left(self, speed: int):
        duty = float(abs(speed))
        if speed > 0:
            self.pwm_in1.ChangeDutyCycle(duty)
            self.pwm_in2.ChangeDutyCycle(0.0)
        elif speed < 0:
            self.pwm_in1.ChangeDutyCycle(0.0)
            self.pwm_in2.ChangeDutyCycle(duty)
        else:
            self.pwm_in1.ChangeDutyCycle(0.0)
            self.pwm_in2.ChangeDutyCycle(0.0)

    def _apply_right(self, speed: int):
        duty = float(abs(speed))
        if speed > 0:
            self.pwm_in3.ChangeDutyCycle(duty)
            self.pwm_in4.ChangeDutyCycle(0.0)
        elif speed < 0:
            self.pwm_in3.ChangeDutyCycle(0.0)
            self.pwm_in4.ChangeDutyCycle(duty)
        else:
            self.pwm_in3.ChangeDutyCycle(0.0)
            self.pwm_in4.ChangeDutyCycle(0.0)

    def forward(self, speed: int = 80):
        """Drives both tracks forward"""
        self.set_motors(speed, speed)

    def backward(self, speed: int = 80):
        """Drives both tracks backward"""
        self.set_motors(-speed, -speed)

    def turn_left(self, speed: int = 75):
        """In-place spin turn to the left"""
        self.set_motors(-speed, speed)

    def turn_right(self, speed: int = 75):
        """In-place spin turn to the right"""
        self.set_motors(speed, -speed)

    def stop(self):
        """Brakes both tracks and turns off all PWM signals"""
        self.set_motors(0, 0)

    def drive_joystick(self, throttle: float, steering: float):
        """
        Differential steering mixing from virtual joystick:
        throttle: -1.0 (reverse) to +1.0 (forward)
        steering: -1.0 (spin left) to +1.0 (spin right)
        """
        left = (throttle + steering) * 100.0
        right = (throttle - steering) * 100.0

        max_mag = max(abs(left), abs(right), 100.0)
        left = (left / max_mag) * 100.0
        right = (right / max_mag) * 100.0

        self.set_motors(int(left), int(right))

    def track_target_error(self, error_x: float, base_speed: int = 50, kp: float = 0.25) -> Tuple[int, int]:
        """
        Proportional closed-loop steering based on horizontal pixel error from vision camera.
        error_x: pixel offset from camera center (-320 to +320)
        Returns computed (left_speed, right_speed)
        """
        steer = max(-40.0, min(40.0, error_x * kp))
        left = int(base_speed + steer)
        right = int(base_speed - steer)
        self.set_motors(left, right)
        return left, right

    def get_status(self) -> dict:
        with self._lock:
            return {
                "left_speed": self.left_speed,
                "right_speed": self.right_speed,
                "is_moving": (self.left_speed != 0 or self.right_speed != 0)
            }
