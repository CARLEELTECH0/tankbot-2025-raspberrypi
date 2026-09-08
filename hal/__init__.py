"""
Tankbot Hardware Abstraction Layer (HAL) Package (v2.0)
Maps STM32 peripheral timing, PWM motor drivers, and serial bus servos to Raspberry Pi 4B.
"""

from hal.motor_chassis import MotorChassis
from hal.arm_servos import ArmServos

__all__ = ["MotorChassis", "ArmServos"]
