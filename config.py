"""
Tankbot Raspberry Pi 4B Configuration
Tailored for replacing the Hiwonder STM32 Core Board via Male-to-Female Jumper Wires.
Directly interfaces with the Hiwonder Tankbot carrier board sockets P8 (Left) & P9 (Right).
"""

import os
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
WEB_DIR = BASE_DIR / "web"

# Network & Web Server
SERVER_CONFIG = {
    "host": "0.0.0.0",
    "port": int(os.getenv("TANKBOT_PORT", 8080)),
    "broadcast_interval_hz": 20,
}

# Motor Controller: Pure 4-Channel PWM (Zero Enable Pins)
# Baseboard ties Enable A and Enable B to 3.3V. IN1, IN2, IN3, IN4 are each independent PWM lines.
MOTOR_CONFIG = {
    # Left Motor (M1) - Dual PWM Direction & Speed
    "in1": 20,            # Pi GPIO 20 (Pin 38) -> Socket P9 Pin 15 (L298N_IN1)
    "in2": 21,            # Pi GPIO 21 (Pin 40) -> Socket P8 Pin 13 (L298N_IN2)

    # Right Motor (M2) - Dual PWM Direction & Speed
    "in3": 26,            # Pi GPIO 26 (Pin 37) -> Socket P8 Pin 14 (L298N_IN3)
    "in4": 16,            # Pi GPIO 16 (Pin 36) -> Socket P8 Pin 15 (L298N_IN4)

    "invert_left": True,
    "invert_right": False,
    "pwm_freq": 1000,     # 1 KHz matching STM32 100us timer loop
    "deadzone": 5,        # Deadband cutoff to prevent motor whine at near-zero
}

# Hiwonder 4-Channel Line Follower Module (I2C)
# 7-bit Address: 0x78 (from STM32 8-bit write address 0xF0 >> 1), Register: 0x01
LINE_FOLLOWER_CONFIG = {
    "i2c_bus": 1,         # Standard Raspberry Pi I2C Bus 1
    "i2c_addr": 0x78,     # 7-bit address (0xF0 >> 1)
    "reg_addr": 0x01,     # Register 0x01 contains 4-channel bitmask [S4 S3 S2 S1]
}

# MPU6050 6-Axis IMU (I2C Bus 1)
IMU_CONFIG = {
    "enabled": True,
    "i2c_bus": 1,
    "i2c_addr": 0x68,
}

# I2C Bus Pin Assignments (Physical Pi 4B Pins 3 & 5)
I2C_CONFIG = {
    "sda_pin": 2,          # Pi GPIO 2 (Pin 3) -> Socket P9 Pin 7 (SDA)
    "scl_pin": 3,          # Pi GPIO 3 (Pin 5) -> Socket P9 Pin 8 (SCL)
    "i2c_bus": 1,
}

# 6-DOF Robotic Arm (Hiwonder 74HC126 Bus Servo Buffer via Sockets P8 & P9)
SERVO_CONFIG = {
    "servo_type": os.getenv("TANKBOT_SERVO_TYPE", "BUS_SERVO"),
    
    # Raspberry Pi 4B UART0
    "serial_port": os.getenv("TANKBOT_SERIAL_PORT", "/dev/serial0"),
    "baudrate": 115200,   # Matches STM32 USART2 baudrate

    # Hardware Buffer Control (74HC126D on carrier board)
    "tx_pin": 14,         # Pi GPIO 14 (Pin 8)  -> Socket P8 Pin 19 (Servo_TX)
    "rx_pin": 15,         # Pi GPIO 15 (Pin 10) -> Socket P8 Pin 20 (Servo_RX)
    "tx_en_pin": 27,      # Pi GPIO 27 (Pin 13) -> Socket P9 Pin 19 (Servo_TX_EN)
    "rx_en_pin": 17,      # Pi GPIO 17 (Pin 11) -> Socket P9 Pin 20 (Servo_RX_EN)

    # Servo Limits and Home Positions (from STM32 main.c)
    "servos": {
        1: {"name": "Base Rotation", "default": 600, "min": 0,   "max": 1000, "pin": 4},
        2: {"name": "Shoulder",      "default": 500, "min": 0,   "max": 1000, "pin": 17},
        3: {"name": "Elbow",         "default": 350, "min": 0,   "max": 1000, "pin": 27},
        4: {"name": "Wrist Pitch",   "default": 100, "min": 0,   "max": 1000, "pin": 22},
        5: {"name": "Wrist Roll",    "default": 300, "min": 0,   "max": 1000, "pin": 10},
        6: {"name": "Gripper Claw",  "default": 500, "min": 125, "max": 875,  "pin": 9},
    },

    "presets": {
        "home": {
            "name": "Home / Ready Pose",
            "positions": {1: 600, 2: 500, 3: 350, 4: 100, 5: 300, 6: 500},
            "duration_ms": 1000
        },
        "rest": {
            "name": "Rest / Folded Pose",
            "positions": {1: 500, 2: 200, 3: 150, 4: 100, 5: 500, 6: 200},
            "duration_ms": 1200
        },
        "wave": {
            "name": "Wave Salute",
            "positions": {1: 500, 2: 750, 3: 500, 4: 300, 5: 700, 6: 500},
            "duration_ms": 800
        }
    }
}

# Ultrasonic Sensor (HC-SR04 via Socket P9)
ULTRASONIC_CONFIG = {
    "trig_pin": 23,        # Pi GPIO 23 (Pin 16) -> Socket P9 Pin 18 (Trig)
    "echo_pin": 24,        # Pi GPIO 24 (Pin 18) -> Socket P9 Pin 17 (Echo via 1k/2k divider)
    "max_distance_cm": 400,
    "timeout_s": 0.03,

    "avoidance_threshold_mm": 290,
    "follow_min_mm": 200,
    "follow_max_mm": 350,
    "follow_detect_mm": 600,
}

# Extra Peripherals on Sockets P8 & P9
EXTRA_PERIPHERALS = {
    "buzzer_pin": 4,       # Pi GPIO 4  (Pin 7)  -> Socket P8 Pin 18 (Buzzer)
    "led_pin": 25,         # Pi GPIO 25 (Pin 22) -> Socket P8 Pin 16 (LED1)
    "key_pin": 22,         # Pi GPIO 22 (Pin 15) -> Socket P9 Pin 5  (KEY)
}

# Safety & Simulation
SAFETY_CONFIG = {
    "heartbeat_timeout_s": 2.5,
    "auto_detect_hardware": True,
    "force_simulation": False,
}
