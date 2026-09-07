"""
Tankbot 4-Channel Line Follower Module
Ported from STM32 linefollow() logic
Reads infrared reflectivity to track black lines on white surfaces.
"""

import logging
from config import LINE_FOLLOWER_CONFIG
from hardware.hal import HAL

logger = logging.getLogger("Tankbot.LineFollower")


class LineFollower:
    """4-Channel IR Line Tracker Sensor Driver"""

    def __init__(self, config: dict = LINE_FOLLOWER_CONFIG):
        self.config = config
        self.hal = HAL()

        self.pins = [
            config.get("left2_pin", 5),
            config.get("left1_pin", 6),
            config.get("right1_pin", 12),
            config.get("right2_pin", 25)
        ]

        for p in self.pins:
            self.hal.setup_input(p, pull_up=True)

        logger.info("Line follower initialized.")

    def read_sensors(self) -> list:
        """Returns 4 binary states: [L2, L1, R1, R2] (0=white/off line, 1=black/on line)"""
        if self.hal.is_simulation:
            return [0, 1, 1, 0]  # Centered on line in simulation

        return [1 if self.hal.read_pin(p) == 0 else 0 for p in self.pins]

    def get_steering_recommendation(self) -> int:
        """
        Calculates differential steering offset based on line position.
        Matches STM32 linefollow() logic:
        - Negative: Steer Left
        - Positive: Steer Right
        - 0: Straight Ahead
        """
        sensors = self.read_sensors()
        l2, l1, r1, r2 = sensors

        if l1 == 1 and r1 == 0:
            return -80  # Steer Left
        elif l1 == 0 and r1 == 1:
            return 80   # Steer Right
        elif l2 == 1 and r2 == 0:
            return -100 # Hard Left
        elif l2 == 0 and r2 == 1:
            return 100  # Hard Right
        else:
            return 0    # Centered
