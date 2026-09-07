"""
Tankbot 6-DOF Robotic Arm Controller
Ported from STM32 BusServoCtrl.c / BusServoCtrl.h / PWM.c
Supports Hiwonder/LewanSoul Serial Bus Servos (LX-16A protocol) and Standard PWM Servos.
"""

import time
import logging
import asyncio
from typing import Dict, Optional, List
from config import SERVO_CONFIG
from hardware.hal import HAL

logger = logging.getLogger("Tankbot.Servo")

# Serial Bus Servo Commands
SERVO_MOVE_TIME_WRITE = 1
SERVO_MOVE_TIME_READ = 2
SERVO_MOVE_TIME_WAIT_WRITE = 7
SERVO_MOVE_TIME_WAIT_READ = 8
SERVO_MOVE_START = 11
SERVO_MOVE_STOP = 12
SERVO_ID_WRITE = 13
SERVO_ID_READ = 14
SERVO_ANGLE_OFFSET_ADJUST = 17
SERVO_ANGLE_OFFSET_WRITE = 18
SERVO_ANGLE_OFFSET_READ = 19
SERVO_ANGLE_LIMIT_WRITE = 20
SERVO_ANGLE_LIMIT_READ = 21
SERVO_VIN_LIMIT_WRITE = 22
SERVO_VIN_LIMIT_READ = 23
SERVO_TEMP_MAX_LIMIT_WRITE = 24
SERVO_TEMP_MAX_LIMIT_READ = 25
SERVO_TEMP_READ = 26
SERVO_VIN_READ = 27
SERVO_POS_READ = 28
SERVO_OR_MOTOR_MODE_WRITE = 29
SERVO_OR_MOTOR_MODE_READ = 30
SERVO_LOAD_OR_UNLOAD_WRITE = 31
SERVO_LOAD_OR_UNLOAD_READ = 32
SERVO_LED_CTRL_WRITE = 33
SERVO_LED_CTRL_READ = 34
SERVO_LED_ERROR_WRITE = 35
SERVO_LED_ERROR_READ = 36


class ServoController:
    """Manages the 6 degrees of freedom of the robotic arm"""

    def __init__(self, config: dict = SERVO_CONFIG):
        self.config = config
        self.hal = HAL()
        self.servo_type = config.get("servo_type", "BUS_SERVO")
        self.servo_defs = config.get("servos", {})
        self.serial_port_name = config.get("serial_port", "/dev/serial0")
        self.baudrate = config.get("baudrate", 115200)

        # Current positions dictionary (ID -> position)
        self.positions: Dict[int, int] = {}
        for servo_id, info in self.servo_defs.items():
            self.positions[servo_id] = info.get("default", 500)

        # Serial bus setup
        self.serial = None
        self._init_serial()

        # Hardware Buffer Control (74HC126 on Hiwonder carrier board)
        self.tx_en_pin = config.get("tx_en_pin", 27)
        self.rx_en_pin = config.get("rx_en_pin", 17)
        if self.tx_en_pin is not None:
            self.hal.setup_output(self.tx_en_pin, initial_high=True)
        if self.rx_en_pin is not None:
            self.hal.setup_output(self.rx_en_pin, initial_high=False)

        # PWM channels (for PWM servo fallback)
        self._pwm_channels = {}
        if self.servo_type == "PWM_SERVO":
            self._init_pwm()

        self._active_sequence_task: Optional[asyncio.Task] = None
        logger.info(f"Servo controller initialized ({self.servo_type} mode)")

    def _init_serial(self):
        try:
            import serial
            self.serial = serial.Serial(self.serial_port_name, self.baudrate, timeout=0.1)
            logger.info(f"Serial port {self.serial_port_name} opened at {self.baudrate} baud")
        except Exception as e:
            logger.info(f"Serial port unavailable ({e}). Using simulated servo response.")
            self.serial = None

    def _init_pwm(self):
        for servo_id, info in self.servo_defs.items():
            pin = info.get("pin")
            if pin is not None:
                pwm = self.hal.get_pwm(pin, frequency=50) # 50Hz standard servo frequency
                pwm.start(0)
                self._pwm_channels[servo_id] = pwm

    def _send_bus_servo_packet(self, servo_id: int, cmd: int, prm1: int, prm2: int):
        """
        Generates and transmits the exact STM32 BusServoCtrl packet:
        [0x55, 0x55, id, data_length, cmd, prm1_l, prm1_h, prm2_l, prm2_h, checksum]
        """
        data_len = 7  # cmd + 4 bytes params + checksum = 7 (matches STM32 SERVO_MOVE_TIME_DATA_LEN)
        prm1_l = prm1 & 0xFF
        prm1_h = (prm1 >> 8) & 0xFF
        prm2_l = prm2 & 0xFF
        prm2_h = (prm2 >> 8) & 0xFF

        packet = bytearray([0x55, 0x55, servo_id, data_len, cmd, prm1_l, prm1_h, prm2_l, prm2_h])
        checksum = (~sum(packet[2:])) & 0xFF
        packet.append(checksum)

        if self.serial and self.serial.is_open:
            try:
                if self.tx_en_pin is not None:
                    self.hal.write_pin(self.tx_en_pin, True)
                if self.rx_en_pin is not None:
                    self.hal.write_pin(self.rx_en_pin, False)
                self.serial.write(packet)
                self.serial.flush()
            except Exception as e:
                logger.error(f"Error writing to servo serial bus: {e}")

    def _update_pwm_servo(self, servo_id: int, position: int):
        """Maps position 0-1000 to standard 50Hz PWM duty cycle (500us - 2500us)"""
        if servo_id in self._pwm_channels:
            # 50Hz period is 20,000us.
            # 0 -> 500us (2.5% duty), 1000 -> 2500us (12.5% duty)
            pulse_us = 500 + (position / 1000.0) * 2000.0
            duty_cycle = (pulse_us / 20000.0) * 100.0
            self._pwm_channels[servo_id].ChangeDutyCycle(duty_cycle)

    def set_servo(self, servo_id: int, position: int, duration_ms: int = 50):
        """
        Sets a single servo position with clamp protection
        Matches STM32 BusServoCtrl(id, SERVO_MOVE_TIME_WRITE, pos, time)
        """
        if servo_id not in self.servo_defs:
            return

        min_pos = self.servo_defs[servo_id].get("min", 0)
        max_pos = self.servo_defs[servo_id].get("max", 1000)
        clamped_pos = max(min_pos, min(max_pos, int(position)))

        self.positions[servo_id] = clamped_pos

        if self.servo_type == "BUS_SERVO":
            self._send_bus_servo_packet(servo_id, SERVO_MOVE_TIME_WRITE, clamped_pos, duration_ms)
        elif self.servo_type == "PWM_SERVO":
            self._update_pwm_servo(servo_id, clamped_pos)

    def set_multiple(self, targets: Dict[int, int], duration_ms: int = 500):
        """Moves multiple servos simultaneously matching STM32 CMD_MULT_SERVO_MOVE"""
        for servo_id, pos in targets.items():
            self.set_servo(servo_id, pos, duration_ms)

    def home(self, duration_ms: int = 1000):
        """Sets all 6 servos to the default home ready position"""
        home_targets = {sid: info["default"] for sid, info in self.servo_defs.items()}
        self.set_multiple(home_targets, duration_ms)

    def open_claw(self, duration_ms: int = 500):
        """Opens the gripper claw (ID 6 to min/open position)"""
        open_pos = self.servo_defs.get(6, {}).get("min", 130)
        self.set_servo(6, open_pos, duration_ms)

    def close_claw(self, duration_ms: int = 500):
        """Closes the gripper claw (ID 6 to closed position)"""
        close_pos = self.servo_defs.get(6, {}).get("max", 850)
        self.set_servo(6, close_pos, duration_ms)

    async def execute_grab_sequence(self):
        """
        Executes the exact 8-step intelligent grabbing sequence from STM32 Control.c
        """
        logger.info("Executing Pick & Place autonomous sequence...")
        
        # Step 1: Initial align
        self.set_multiple({1: 300, 2: 500, 3: 350, 4: 100, 5: 300, 6: 500}, 1000)
        await asyncio.sleep(1.0)

        # Step 2: Reach forward
        self.set_multiple({1: 140, 2: 500, 3: 810, 4: 100, 5: 765}, 1000)
        await asyncio.sleep(1.5)

        # Step 3: Base align
        self.set_servo(1, 600, 1000)
        await asyncio.sleep(1.0)

        # Step 4: Lift slightly
        self.set_multiple({3: 350, 5: 300}, 1000)
        await asyncio.sleep(1.0)

        # Step 5: Close gripper on object
        self.set_servo(6, 130, 1000)
        await asyncio.sleep(1.0)

        # Step 6: Lift payload
        self.set_multiple({3: 810, 5: 765}, 1000)
        await asyncio.sleep(1.2)

        # Step 7: Rotate payload to storage / place position
        self.set_servo(1, 0, 1000)
        await asyncio.sleep(1.2)

        # Step 8: Lower and release
        self.set_multiple({3: 350, 5: 300}, 1000)
        await asyncio.sleep(1.0)
        self.set_servo(1, 300, 1000)
        self.set_servo(6, 500, 1000)
        await asyncio.sleep(1.0)
        logger.info("Pick & Place sequence finished.")

    def get_positions(self) -> Dict[int, int]:
        return dict(self.positions)
