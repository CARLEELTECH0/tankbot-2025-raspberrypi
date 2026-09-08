"""
Tankbot Telemetry Data Model
Standardized status reporting for WebSocket streaming and Mobile UI rendering.
"""

from dataclasses import dataclass, asdict, field
from typing import Dict, List, Optional
import time


@dataclass
class TelemetryData:
    timestamp: float = field(default_factory=time.time)
    mode: str = "MANUAL"
    emergency_stopped: bool = False
    is_simulation: bool = False

    # Motor Status
    left_speed: int = 0
    right_speed: int = 0
    is_moving: bool = False

    # Arm Servos (ID 1-6 positions: 0-1000)
    servo_positions: Dict[int, int] = field(default_factory=dict)
    active_sequence: Optional[str] = None

    # Sensors
    distance_mm: int = 500
    distance_cm: float = 50.0
    line_sensors: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    pitch: float = 0.0
    roll: float = 0.0
    is_level: bool = True

    # Battery & System Health
    battery_mv: int = 7400  # 2S LiPo nominal ~7.4V
    battery_pct: int = 85
    heartbeat_ok: bool = True

    # Computer Vision & Color Tracking (v2.0)
    vision_enabled: bool = True
    vision_target_color: str = "red"
    vision_detected: bool = False
    vision_coords: Dict[str, float] = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 2.0})
    vision_rotation_angle: float = 0.0
    vision_wrist_pulse: int = 500
    vision_status: str = "SEARCHING"
    camera_fps: float = 0.0

    def to_dict(self) -> dict:
        d = asdict(self)
        d["timestamp"] = round(self.timestamp, 3)
        return d
