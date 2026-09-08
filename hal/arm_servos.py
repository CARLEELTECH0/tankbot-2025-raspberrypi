"""
Tankbot 6-DOF Bus Servo Controller - Hardware Abstraction Layer
Ported from STM32 BusServoCtrl.c and ArmPi HiwonderSDK/Board.py / BusServoCmd.py.
Handles high-speed serial packet generation (115200 baud) over /dev/serial0 or /dev/ttyAMA0
with 74HC126D hardware directional buffer control (GPIO 27 TX_EN, GPIO 17 RX_EN).
"""

import time
import logging
import threading
from typing import Dict, Optional
from config import SERVO_CONFIG
from hardware.hal import HAL

logger = logging.getLogger("Tankbot.HAL.Servos")

# Hiwonder / Lobot LX-16A Protocol Constants
LOBOT_SERVO_FRAME_HEADER         = 0x55
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_TIME_READ       = 2
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_ID_WRITE             = 13
LOBOT_SERVO_ID_READ              = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   = 18
LOBOT_SERVO_POS_READ             = 28


class ArmServos:
    """
    Hardware Abstraction for the 6-DOF Serial Bus Robotic Arm.
    Controls joints 1 to 6 with hardware safety clamps and buffer management.
    """

    def __init__(self, config: Optional[dict] = None):
        self.config = config or SERVO_CONFIG
        self.hal = HAL()

        self.serial_port_name = self.config.get("serial_port", "/dev/serial0")
        self.baudrate = self.config.get("baudrate", 115200)
        self.servo_defs = self.config.get("servos", {})

        # Hardware Buffer Control Pins (74HC126 on carrier board)
        self.tx_en_pin = self.config.get("tx_en_pin", 27)  # Socket P9 Pin 19
        self.rx_en_pin = self.config.get("rx_en_pin", 17)  # Socket P9 Pin 20

        self._lock = threading.Lock()
        self.serial = None
        self.is_connected = False

        # Current positions cache (ID 1-6 -> pulse 0-1000)
        self.positions: Dict[int, int] = {}
        for sid, info in self.servo_defs.items():
            self.positions[sid] = info.get("default", 500)

        self._init_pins()
        self._init_serial()
        logger.info(f"ArmServos initialized on {self.serial_port_name} @ {self.baudrate} baud.")

    def _init_pins(self):
        """Initializes 74HC126 buffer direction control pins"""
        if self.tx_en_pin is not None:
            self.hal.setup_output(self.tx_en_pin, initial_high=True)
        if self.rx_en_pin is not None:
            self.hal.setup_output(self.rx_en_pin, initial_high=False)

    def _init_serial(self):
        """Opens UART serial port"""
        try:
            import serial
            ports_to_try = [self.serial_port_name, "/dev/ttyAMA0", "/dev/ttyS0", "/dev/serial0"]
            for port in ports_to_try:
                try:
                    self.serial = serial.Serial(port, self.baudrate, timeout=0.05)
                    self.serial_port_name = port
                    self.is_connected = True
                    logger.info(f"Connected to bus servos via serial port: {port}")
                    break
                except Exception:
                    continue

            if not self.is_connected:
                logger.info("Hardware serial port unavailable. Using simulated servo control.")
        except ImportError:
            logger.info("pyserial not available. Running in simulated servo mode.")

    def _send_packet(self, servo_id: int, cmd: int, prm1: int, prm2: int):
        """
        Transmits a protocol packet:
        [0x55, 0x55, id, length, cmd, prm1_l, prm1_h, prm2_l, prm2_h, checksum]
        """
        data_len = 7
        p1_l = prm1 & 0xFF
        p1_h = (prm1 >> 8) & 0xFF
        p2_l = prm2 & 0xFF
        p2_h = (prm2 >> 8) & 0xFF

        packet = bytearray([
            LOBOT_SERVO_FRAME_HEADER,
            LOBOT_SERVO_FRAME_HEADER,
            servo_id & 0xFF,
            data_len,
            cmd,
            p1_l,
            p1_h,
            p2_l,
            p2_h
        ])
        checksum = (~sum(packet[2:])) & 0xFF
        packet.append(checksum)

        if self.serial and self.serial.is_open:
            try:
                # Gating: enable TX, disable RX
                if self.tx_en_pin is not None:
                    self.hal.write_pin(self.tx_en_pin, True)
                if self.rx_en_pin is not None:
                    self.hal.write_pin(self.rx_en_pin, False)

                self.serial.write(packet)
                self.serial.flush()
            except Exception as e:
                logger.debug(f"Serial transmission error: {e}")

    def set_servo_pulse(self, servo_id: int, pulse: int, use_time_ms: int = 50):
        """
        Drives a single servo to target pulse (0-1000).
        Ported from ArmPi Board.setBusServoPulse(id, pulse, use_time).
        """
        with self._lock:
            # Enforce hardware range limits
            min_limit = self.servo_defs.get(servo_id, {}).get("min", 0)
            max_limit = self.servo_defs.get(servo_id, {}).get("max", 1000)
            clamped_pulse = max(min_limit, min(max_limit, int(pulse)))
            clamped_time = max(0, min(30000, int(use_time_ms)))

            self.positions[servo_id] = clamped_pulse
            self._send_packet(servo_id, LOBOT_SERVO_MOVE_TIME_WRITE, clamped_pulse, clamped_time)

    def set_multiple_servos(self, targets: Dict[int, int], use_time_ms: int = 500):
        """
        Synchronously updates multiple servos.
        targets: Dict mapping servo_id (1-6) -> pulse (0-1000)
        """
        for sid, pulse in targets.items():
            self.set_servo_pulse(sid, pulse, use_time_ms)

    def home(self, use_time_ms: int = 1000):
        """Returns arm to default safe home pose"""
        home_targets = {sid: info.get("default", 500) for sid, info in self.servo_defs.items()}
        self.set_multiple_servos(home_targets, use_time_ms)

    def open_claw(self, use_time_ms: int = 500):
        """Opens the gripper claw (ID 6 to min open limit ~200)"""
        open_pulse = self.servo_defs.get(6, {}).get("min", 130)
        self.set_servo_pulse(6, open_pulse, use_time_ms)

    def close_claw(self, use_time_ms: int = 500):
        """Closes the gripper claw (ID 6 to closed limit ~500)"""
        close_pulse = self.servo_defs.get(6, {}).get("default", 500)
        self.set_servo_pulse(6, close_pulse, use_time_ms)

    def stop_servo(self, servo_id: Optional[int] = None):
        """Stops servo motion"""
        with self._lock:
            sid = 0xFE if servo_id is None else servo_id
            self._send_packet(sid, LOBOT_SERVO_MOVE_STOP, 0, 0)

    def get_positions(self) -> Dict[int, int]:
        """Returns snapshot of current cached servo positions"""
        with self._lock:
            return self.positions.copy()
