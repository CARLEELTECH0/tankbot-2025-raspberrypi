"""
Tankbot Hiwonder 4-Channel Line Follower Module (I2C)
Ported from STM32 Lesson 4 Line Following (IIC.c / main.c)
Communicates over I2C Bus 1 at 7-bit address 0x78 (from STM32 8-bit address 0xF0).
Reads register 0x01 containing the 4-channel infrared sensor state [S4 S3 S2 S1].
"""

import logging
from typing import List, Tuple
from config import LINE_FOLLOWER_CONFIG
from hardware.hal import HAL

logger = logging.getLogger("Tankbot.LineFollower")


class LineFollower:
    """Hiwonder 4-Channel I2C Line Tracker Sensor Driver"""

    def __init__(self, config: dict = LINE_FOLLOWER_CONFIG):
        self.config = config
        self.hal = HAL()

        self.i2c_bus_num = config.get("i2c_bus", 1)
        self.i2c_addr = config.get("i2c_addr", 0x78)  # 0xF0 >> 1
        self.reg_addr = config.get("reg_addr", 0x01)

        self.bus = None
        self.is_connected = False
        self._last_raw_val = 0x06  # Default: Center two sensors active [0, 1, 1, 0]

        self._init_i2c()

    def _init_i2c(self):
        if not self.hal.is_simulation:
            try:
                import smbus2
                self.bus = smbus2.SMBus(self.i2c_bus_num)
                # Test read to verify device is present on I2C bus
                _ = self.bus.read_byte_data(self.i2c_addr, self.reg_addr)
                self.is_connected = True
                logger.info(f"Hiwonder 4-Channel Line Follower connected at I2C 0x{self.i2c_addr:02X}.")
            except Exception as e:
                logger.info(f"Hiwonder I2C Line Follower not detected ({e}). Using simulation mode.")
                self.is_connected = False
        else:
            logger.info("Hiwonder Line Follower running in simulation mode.")

    def read_raw_byte(self) -> int:
        """
        Reads 1 byte from I2C register 0x01.
        Matches STM32:
        IIC_send_byte(0xF0); IIC_send_byte(0x01); IIC_send_byte(0xF1); val = IIC_read_byte(0);
        """
        if self.is_connected and self.bus is not None:
            try:
                self._last_raw_val = self.bus.read_byte_data(self.i2c_addr, self.reg_addr)
            except Exception as e:
                logger.debug(f"I2C read error from Line Follower: {e}")

        return self._last_raw_val

    def read_sensors(self) -> List[int]:
        """
        Returns 4 binary sensor states: [Sensor 1, Sensor 2, Sensor 3, Sensor 4]
        - 1: Black line detected
        - 0: White surface
        Left to right corresponds to Sensor 1 through Sensor 4.
        """
        val = self.read_raw_byte()
        s1 = val & 0x01          # Bit 0: Far Left
        s2 = (val >> 1) & 0x01   # Bit 1: Center Left
        s3 = (val >> 2) & 0x01   # Bit 2: Center Right
        s4 = (val >> 3) & 0x01   # Bit 3: Far Right
        return [s1, s2, s3, s4]

    def get_motor_speeds(self, base_speed: int = 80) -> Tuple[int, int]:
        """
        Calculates left and right track motor speeds based on I2C line sensor readings.
        Directly implements the exact control logic from STM32 Lesson 4 Line Following:
        - S2=0 & S3=1: Turn Right -> MotorControl(80, -80)
        - S2=1 & S3=0: Turn Left  -> MotorControl(-80, 80)
        - S1=1, S2=1, S3=1, S4=1: Stop Line -> MotorControl(0, 0)
        - Otherwise: Straight Forward -> MotorControl(80, 80)
        """
        val = self.read_raw_byte()
        s1 = val & 0x01
        s2 = (val >> 1) & 0x01
        s3 = (val >> 2) & 0x01
        s4 = (val >> 3) & 0x01

        # Stop line: all 4 sensors on black line
        if s1 == 1 and s2 == 1 and s3 == 1 and s4 == 1:
            return (0, 0)

        # Center-left on line, center-right off -> robot drifted right, steer LEFT
        elif s2 == 1 and s3 == 0:
            return (-base_speed, base_speed)

        # Center-left off line, center-right on -> robot drifted left, steer RIGHT
        elif s2 == 0 and s3 == 1:
            return (base_speed, -base_speed)

        # Far left triggered -> sharp LEFT turn
        elif s1 == 1 and s4 == 0:
            return (-base_speed, base_speed)

        # Far right triggered -> sharp RIGHT turn
        elif s1 == 0 and s4 == 1:
            return (base_speed, -base_speed)

        # Centered on line (both S2 & S3 detect line) or searching -> drive forward
        else:
            return (base_speed, base_speed)

    def get_steering_recommendation(self) -> float:
        """
        Calculates a normalized steering hint:
        -1.0 (turn left), 0.0 (straight), +1.0 (turn right)
        """
        val = self.read_raw_byte()
        s1 = val & 0x01
        s2 = (val >> 1) & 0x01
        s3 = (val >> 2) & 0x01
        s4 = (val >> 3) & 0x01

        if (s2 == 1 and s3 == 0) or (s1 == 1 and s4 == 0):
            return -1.0
        elif (s2 == 0 and s3 == 1) or (s1 == 0 and s4 == 1):
            return 1.0
        return 0.0

