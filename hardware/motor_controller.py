"""
Tankbot Dual Track Motor Controller - Pure 4-PWM Architecture
Ported from STM32 Motor.c / Motor.h
Directly drives IN1, IN2 (Left Motor M1) and IN3, IN4 (Right Motor M2) as independent PWM channels.
No enable pins required (matches Hiwonder baseboard L298P with Enable A & B tied to 3.3V).
"""

import logging
from config import MOTOR_CONFIG
from hardware.hal import HAL

logger = logging.getLogger("Tankbot.Motor")


class MotorController:
    """Controls the differential drive track motors using 4 independent PWM channels"""

    def __init__(self, config: dict = MOTOR_CONFIG):
        self.config = config
        self.hal = HAL()

        self.in1_pin = config.get("in1", 20)
        self.in2_pin = config.get("in2", 21)
        self.in3_pin = config.get("in3", 26)
        self.in4_pin = config.get("in4", 16)

        self.invert_left = config.get("invert_left", False)
        self.invert_right = config.get("invert_right", False)
        self.deadzone = config.get("deadzone", 5)
        self.pwm_freq = config.get("pwm_freq", 1000)

        # Initialize all 4 pins as dedicated PWM outputs (1 kHz)
        self.pwm_in1 = self.hal.get_pwm(self.in1_pin, self.pwm_freq)
        self.pwm_in2 = self.hal.get_pwm(self.in2_pin, self.pwm_freq)
        self.pwm_in3 = self.hal.get_pwm(self.in3_pin, self.pwm_freq)
        self.pwm_in4 = self.hal.get_pwm(self.in4_pin, self.pwm_freq)

        self.pwm_in1.start(0)
        self.pwm_in2.start(0)
        self.pwm_in3.start(0)
        self.pwm_in4.start(0)

        self.current_left_speed = 0
        self.current_right_speed = 0
        self.stop()
        logger.info("Motor controller initialized with 4 pure PWM channels (Zero Enable Pins).")

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

        self.current_left_speed = m1_speed
        self.current_right_speed = m2_speed

        drive_m1 = -m1_speed if self.invert_left else m1_speed
        drive_m2 = -m2_speed if self.invert_right else m2_speed

        self._drive_left(drive_m1)
        self._drive_right(drive_m2)

    def _drive_left(self, speed: int):
        """
        Left Motor (M1) PWM Control:
        - Forward: IN1 = PWM(speed), IN2 = 0
        - Reverse: IN1 = 0, IN2 = PWM(speed)
        - Stop/Brake: IN1 = 0, IN2 = 0
        """
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

    def _drive_right(self, speed: int):
        """
        Right Motor (M2) PWM Control:
        - Forward: IN3 = PWM(speed), IN4 = 0
        - Reverse: IN3 = 0, IN4 = PWM(speed)
        - Stop/Brake: IN3 = 0, IN4 = 0
        """
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
        """Emergency and normal stop: sets all 4 PWM outputs to 0"""
        self.set_motors(0, 0)

    def get_status(self) -> dict:
        return {
            "left_speed": self.current_left_speed,
            "right_speed": self.current_right_speed,
            "is_moving": (self.current_left_speed != 0 or self.current_right_speed != 0)
        }
