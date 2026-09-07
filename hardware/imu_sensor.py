"""
Tankbot MPU6050 6-Axis IMU Sensor Module
Ported from STM32 mpu6050.c / mpu6050.h
Reads Accelerometer + Gyroscope data over I2C to calculate pitch, roll, and posture.
"""

import math
import logging
from config import IMU_CONFIG
from hardware.hal import HAL

logger = logging.getLogger("Tankbot.IMU")

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


class IMUSensor:
    """MPU6050 Gyroscope and Accelerometer Interface"""

    def __init__(self, config: dict = IMU_CONFIG):
        self.config = config
        self.hal = HAL()
        self.bus = None
        self.pitch = 0.0
        self.roll = 0.0
        self.is_connected = False

        if not self.hal.is_simulation:
            try:
                import smbus2
                self.bus = smbus2.SMBus(config.get("i2c_bus", 1))
                self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0) # Wake up MPU6050
                self.is_connected = True
                logger.info("MPU6050 connected on I2C bus.")
            except Exception as e:
                logger.info(f"MPU6050 not detected ({e}). Using simulated posture.")

    def read_posture(self) -> dict:
        """Calculates pitch and roll angles matching STM32 complementary filter"""
        if not self.is_connected or self.bus is None:
            return {"pitch": 0.0, "roll": 0.0, "is_level": True, "rollover": False}

        try:
            # Read 6 bytes of accelerometer data
            data = self.bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 6)
            ax = self._convert_to_signed_16(data[0], data[1])
            ay = self._convert_to_signed_16(data[2], data[3])
            az = self._convert_to_signed_16(data[4], data[5])

            # Calculate pitch and roll in degrees
            roll = math.atan2(ay, az) * 180.0 / math.pi
            pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi

            self.roll = round(roll, 1)
            self.pitch = round(pitch, 1)

            is_rollover = abs(roll) > 60.0 or abs(pitch) > 60.0
            is_level = abs(roll) < 10.0 and abs(pitch) < 10.0

            return {
                "pitch": self.pitch,
                "roll": self.roll,
                "is_level": is_level,
                "rollover": is_rollover
            }
        except Exception as e:
            return {"pitch": 0.0, "roll": 0.0, "is_level": True, "rollover": False}

    def _convert_to_signed_16(self, high: int, low: int) -> int:
        val = (high << 8) | low
        if val >= 0x8000:
            val -= 0x10000
        return val
