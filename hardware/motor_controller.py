"""
Tankbot Dual Track Motor Controller
Ported from STM32 Motor.c / Motor.h
Controls Left Track (M1) and Right Track (M2) with PWM speed and directional logic.
"""

import logging
from typing import Tuple
from config import MOTOR_CONFIG
from hardware.hal import HAL

logger = logging.getLogger("Tankbot.Motor")


class MotorController:
    """Controls the differential drive track motors of the Tankbot"""

    def __init__(self, config: dict = MOTOR_CONFIG):
        self.config = config
        self.hal = HAL()

        self.in1 = config["left_in1"]
        self.in2 = config["left_in2"]
        self.in3 = config["right_in1"]
        self.in4 = config["right_in2"]
        self.pwm_left_pin = config.get("left_pwm")
        self.pwm_right_pin = config.get("right_pwm")

        self.invert_left = config.get("invert_left", False)
        self.invert_right = config.get("invert_right", False)
        self.deadzone = config.get("deadzone", 5)
        self.pwm_freq = config.get("pwm_freq", 1000)

        # Setup pins
        self.hal.setup_output(self.in1)
        self.hal.setup_output(self.in2)
        self.hal.setup_output(self.in3)
        self.hal.setup_output(self.in4)

        # PWM setup
        self.left_pwm = None
        self.right_pwm = None
        if self.pwm_left_pin:
            self.left_pwm = self.hal.get_pwm(self.pwm_left_pin, self.pwm_freq)
            self.left_pwm.start(0)
        else:
            self.pwm_in1 = self.hal.get_pwm(self.in1, self.pwm_freq)
            self.pwm_in2 = self.hal.get_pwm(self.in2, self.pwm_freq)
            self.pwm_in1.start(0)
            self.pwm_in2.start(0)

        if self.pwm_right_pin:
            self.right_pwm = self.hal.get_pwm(self.pwm_right_pin, self.pwm_freq)
            self.right_pwm.start(0)
        else:
            self.pwm_in3 = self.hal.get_pwm(self.in3, self.pwm_freq)
            self.pwm_in4 = self.hal.get_pwm(self.in4, self.pwm_freq)
            self.pwm_in3.start(0)
            self.pwm_in4.start(0)

        self.current_left_speed = 0
        self.current_right_speed = 0
        self.stop()
        logger.info("Motor controller initialized successfully.")

    def set_motors(self, m1_speed: int, m2_speed: int):
        """
        Direct Left/Right motor speed setting (-100 to +100)
        Matches STM32 MotorControl(int8 m1Speed, int8 m2Speed)
        """
        m1_speed = max(-100, min(100, int(m1_speed)))
        m2_speed = max(-100, min(100, int(m2_speed)))

        if abs(m1_speed) < self.deadzone:
            m1_speed = 0
        if abs(m2_speed) < self.deadzone:
            m2_speed = 0

        if self.invert_left:
            m1_speed = -m1_speed
        if self.invert_right:
            m2_speed = -m2_speed

        self.current_left_speed = m1_speed
        self.current_right_speed = m2_speed

        self._drive_left(m1_speed)
        self._drive_right(m2_speed)

    def _drive_left(self, speed: int):
        duty = abs(speed)
        if self.pwm_left_pin and self.left_pwm:
            if speed > 0:
                self.hal.write_pin(self.in1, True)
                self.hal.write_pin(self.in2, False)
            elif speed < 0:
                self.hal.write_pin(self.in1, False)
                self.hal.write_pin(self.in2, True)
            else:
                self.hal.write_pin(self.in1, False)
                self.hal.write_pin(self.in2, False)
            self.left_pwm.ChangeDutyCycle(duty)
        else:
            # PWM directly on direction pins
            if speed > 0:
                self.pwm_in1.ChangeDutyCycle(duty)
                self.pwm_in2.ChangeDutyCycle(0)
            elif speed < 0:
                self.pwm_in1.ChangeDutyCycle(0)
                self.pwm_in2.ChangeDutyCycle(duty)
            else:
                self.pwm_in1.ChangeDutyCycle(0)
                self.pwm_in2.ChangeDutyCycle(0)

    def _drive_right(self, speed: int):
        duty = abs(speed)
        if self.pwm_right_pin and self.right_pwm:
            if speed > 0:
                self.hal.write_pin(self.in3, True)
                self.hal.write_pin(self.in4, False)
            elif speed < 0:
                self.hal.write_pin(self.in3, False)
                self.hal.write_pin(self.in4, True)
            else:
                self.hal.write_pin(self.in3, False)
                self.hal.write_pin(self.in4, False)
            self.right_pwm.ChangeDutyCycle(duty)
        else:
            if speed > 0:
                self.pwm_in3.ChangeDutyCycle(duty)
                self.pwm_in4.ChangeDutyCycle(0)
            elif speed < 0:
                self.pwm_in3.ChangeDutyCycle(0)
                self.pwm_in4.ChangeDutyCycle(duty)
            else:
                self.pwm_in3.ChangeDutyCycle(0)
                self.pwm_in4.ChangeDutyCycle(0)

    def drive_joystick(self, throttle: float, steering: float):
        """
        Differential steering mixing from virtual joystick:
        throttle: -1.0 (reverse) to +1.0 (forward)
        steering: -1.0 (spin left) to +1.0 (spin right)
        """
        left = (throttle + steering) * 100.0
        right = (throttle - steering) * 100.0

        # Scale down if out of range to preserve steering ratio
        max_val = max(abs(left), abs(right), 100.0)
        left = (left / max_val) * 100.0
        right = (right / max_val) * 100.0

        self.set_motors(int(left), int(right))

    def forward(self, speed: int = 80):
        self.set_motors(speed, speed)

    def backward(self, speed: int = 80):
        self.set_motors(-speed, -speed)

    def turn_left(self, speed: int = 80):
        self.set_motors(-speed, speed)

    def turn_right(self, speed: int = 80):
        self.set_motors(speed, -speed)

    def stop(self):
        """Emergency and normal stop"""
        self.set_motors(0, 0)

    def get_status(self) -> dict:
        return {
            "left_speed": self.current_left_speed,
            "right_speed": self.current_right_speed,
            "is_moving": (self.current_left_speed != 0 or self.current_right_speed != 0)
        }
