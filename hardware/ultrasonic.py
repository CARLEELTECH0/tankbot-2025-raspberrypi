"""
Tankbot Ultrasonic Sensor Module (HC-SR04)
Ported from STM32 ChaoShengBo.c / ChaoShengBo.h
Accurate pulse-timing distance measurement with noise filtering and simulation physics.
"""

import time
import logging
from collections import deque
from config import ULTRASONIC_CONFIG
from hardware.hal import HAL

logger = logging.getLogger("Tankbot.Ultrasonic")


class UltrasonicSensor:
    """HC-SR04 Ultrasonic Distance Sensor Driver"""

    def __init__(self, config: dict = ULTRASONIC_CONFIG):
        self.config = config
        self.hal = HAL()
        self.trig_pin = config["trig_pin"]
        self.echo_pin = config["echo_pin"]
        self.max_distance_cm = config.get("max_distance_cm", 400)
        self.timeout_s = config.get("timeout_s", 0.03)

        self.hal.setup_output(self.trig_pin, initial_high=False)
        self.hal.setup_input(self.echo_pin, pull_up=False)

        self._readings_buffer = deque(maxlen=5)
        self._last_valid_distance_mm = 500  # Default 50cm
        self._sim_distance_cm = 45.0
        self._last_measure_time = 0.0

        logger.info("Ultrasonic sensor initialized.")

    def measure_distance_mm(self, sim_motor_speed: int = 0) -> int:
        """
        Measures distance in millimeters.
        Matches STM32 GetDistance() calculation:
        mm = us * 17 / 100
        """
        if self.hal.is_simulation:
            # Simulate real-world motion based on motor direction
            if sim_motor_speed > 0:
                self._sim_distance_cm = max(10.0, self._sim_distance_cm - 1.2)
            elif sim_motor_speed < 0:
                self._sim_distance_cm = min(150.0, self._sim_distance_cm + 1.2)
            else:
                # Slight sensor jitter
                import random
                self._sim_distance_cm = max(12.0, min(140.0, self._sim_distance_cm + random.uniform(-0.3, 0.3)))
            return int(self._sim_distance_cm * 10)

        # Physical hardware measurement
        now = time.time()
        if now - self._last_measure_time < 0.04:
            # Enforce 40ms minimum echo decay time
            time.sleep(0.04 - (now - self._last_measure_time))
        self._last_measure_time = time.time()

        try:
            # Send 10us trigger pulse
            self.hal.write_pin(self.trig_pin, False)
            time.sleep(0.000002)
            self.hal.write_pin(self.trig_pin, True)
            time.sleep(0.000010)
            self.hal.write_pin(self.trig_pin, False)

            # Wait for echo to go HIGH
            start_wait = time.time()
            while self.hal.read_pin(self.echo_pin) == 0:
                if time.time() - start_wait > self.timeout_s:
                    return self._last_valid_distance_mm
            pulse_start = time.time()

            # Wait for echo to go LOW
            while self.hal.read_pin(self.echo_pin) == 1:
                if time.time() - pulse_start > self.timeout_s:
                    return self._last_valid_distance_mm
            pulse_end = time.time()

            duration_us = (pulse_end - pulse_start) * 1_000_000
            
            # 340 m/s = 0.34 mm/us -> round trip / 2 = 0.17 mm/us
            distance_mm = int(duration_us * 17 / 100)

            # Sanity checks
            if 20 <= distance_mm <= (self.max_distance_cm * 10):
                self._readings_buffer.append(distance_mm)
                # Median filter
                sorted_vals = sorted(self._readings_buffer)
                median_dist = sorted_vals[len(sorted_vals) // 2]
                self._last_valid_distance_mm = median_dist
                return median_dist
            else:
                return self._last_valid_distance_mm

        except Exception as e:
            logger.debug(f"Ultrasonic measurement error: {e}")
            return self._last_valid_distance_mm

    def measure_distance_cm(self, sim_motor_speed: int = 0) -> float:
        return round(self.measure_distance_mm(sim_motor_speed) / 10.0, 1)
