"""
Tankbot USB Camera Processor
Handles standard OpenCV (cv2.VideoCapture) video acquisition over USB-A on Raspberry Pi 4B.
Targets /dev/video0 at 640x480 resolution @ 30 FPS with dedicated capture threading,
double-buffering, and synthetic test-bench simulation fallback.
"""

import time
import logging
import threading
from typing import Optional, Tuple
import numpy as np

logger = logging.getLogger("Tankbot.Camera")


class CameraProcessor:
    """
    High-performance asynchronous USB Camera capture engine.
    Runs video capture in a background thread to prevent blocking robot motion loops
    and drops stale frames so downstream CV algorithms always process real-time imagery.
    """

    def __init__(
        self,
        device: str = "/dev/video0",
        resolution: Tuple[int, int] = (640, 480),
        target_fps: int = 30,
        fourcc: str = "MJPG"
    ):
        self.device = device
        self.width = resolution[0]
        self.height = resolution[1]
        self.target_fps = target_fps
        self.fourcc_str = fourcc

        self.cap = None
        self.is_opened = False
        self.is_simulated = False

        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # FPS calculation
        self._fps: float = 0.0
        self._frame_count = 0
        self._last_fps_time = time.time()

        # Synthetic generator variables (for simulation mode)
        self._sim_theta = 0.0
        self._sim_color = "red"

    def start(self) -> bool:
        """Starts the background capture loop"""
        if self._running:
            return True

        self._running = True
        success = self._open_hardware_camera()

        if not success:
            logger.warning(f"Could not open hardware camera '{self.device}'. Falling back to Synthetic Vision Simulation.")
            self.is_simulated = True
            self.is_opened = True
            self._frame = self._generate_synthetic_frame()

        self._thread = threading.Thread(target=self._capture_worker, daemon=True, name="CameraCaptureWorker")
        self._thread.start()
        logger.info(f"Camera processor started (Resolution: {self.width}x{self.height} @ {self.target_fps} FPS, Simulation: {self.is_simulated})")
        return True

    def _open_hardware_camera(self) -> bool:
        """Attempts to open USB camera via OpenCV"""
        try:
            import cv2
        except ImportError:
            logger.info("OpenCV (cv2) not installed yet. Using simulation mode.")
            return False

        try:
            # Parse integer index or path
            dev_target = self.device
            if isinstance(dev_target, str) and dev_target.isdigit():
                dev_target = int(dev_target)
            elif isinstance(dev_target, str) and dev_target.startswith("/dev/video"):
                try:
                    dev_target = int(dev_target.replace("/dev/video", ""))
                except ValueError:
                    pass

            self.cap = cv2.VideoCapture(dev_target)
            if not self.cap.isOpened():
                # Fallback to index 0
                self.cap = cv2.VideoCapture(0)

            if not self.cap.isOpened():
                return False

            # Lock resolution to 640x480
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)

            # MJPG fourcc significantly reduces USB bus congestion on Raspberry Pi 4B
            try:
                fourcc_code = cv2.VideoWriter_fourcc(*self.fourcc_str)
                self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_code)
            except Exception:
                pass

            # Read a test frame to verify sensor stream
            ret, test_frame = self.cap.read()
            if ret and test_frame is not None:
                self.is_opened = True
                self.is_simulated = False
                logger.info(f"Hardware USB camera successfully locked to {self.width}x{self.height} @ {self.target_fps} FPS on {self.device}")
                return True
            else:
                self.cap.release()
                self.cap = None
                return False

        except Exception as e:
            logger.debug(f"Failed to open USB camera: {e}")
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
            return False

    def _capture_worker(self):
        """Worker thread that continuously captures or generates frames"""
        frame_interval = 1.0 / max(1, self.target_fps)

        while self._running:
            loop_start = time.time()

            frame = None
            if not self.is_simulated and self.cap is not None and self.cap.isOpened():
                try:
                    ret, raw_frame = self.cap.read()
                    if ret and raw_frame is not None:
                        # Resize if camera did not conform to requested resolution
                        if raw_frame.shape[1] != self.width or raw_frame.shape[0] != self.height:
                            import cv2
                            frame = cv2.resize(raw_frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
                        else:
                            frame = raw_frame
                    else:
                        # Device disconnected or frame dropped; retry
                        time.sleep(0.01)
                except Exception as e:
                    logger.debug(f"Frame capture error: {e}")
                    time.sleep(0.05)

            if frame is None:
                # Generate synthetic test frame
                frame = self._generate_synthetic_frame()

            # Store latest frame with lock
            with self._lock:
                self._frame = frame

            # Calculate rolling FPS
            self._frame_count += 1
            now = time.time()
            elapsed = now - self._last_fps_time
            if elapsed >= 1.0:
                self._fps = round(self._frame_count / elapsed, 1)
                self._frame_count = 0
                self._last_fps_time = now

            # Throttle to target FPS
            sleep_duration = frame_interval - (time.time() - loop_start)
            if sleep_duration > 0.001:
                time.sleep(sleep_duration)

    def _generate_synthetic_frame(self) -> np.ndarray:
        """
        Generates a realistic 640x480 synthetic test frame containing:
        - Workbench grid pattern
        - Moving colored target block (red/green/blue)
        - Timestamp & camera status overlay
        Allows full CV testing even without physical USB hardware.
        """
        # Create dark workbench background
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[:] = (35, 30, 30)

        # Draw grid lines
        grid_step = 40
        for x in range(0, self.width, grid_step):
            color = (50, 45, 45) if x != self.width // 2 else (70, 70, 70)
            frame[:, x] = color
        for y in range(0, self.height, grid_step):
            color = (50, 45, 45) if y != self.height // 2 else (70, 70, 70)
            frame[y, :] = color

        # Compute orbiting target position
        self._sim_theta += 0.04
        center_x = int(self.width / 2 + 160 * np.cos(self._sim_theta))
        center_y = int(self.height / 2 + 80 * np.sin(self._sim_theta * 1.5))
        box_size = 50

        # Draw simulated colored object (BGR)
        bgr_colors = {
            "red": (30, 30, 220),
            "green": (40, 220, 40),
            "blue": (220, 60, 40)
        }
        bgr = bgr_colors.get(self._sim_color, (30, 30, 220))

        # Rotate square
        try:
            import cv2
            angle_deg = (self._sim_theta * 45) % 360
            rect = ((center_x, center_y), (box_size, box_size), angle_deg)
            box = np.intp(cv2.boxPoints(rect))
            cv2.drawContours(frame, [box], 0, bgr, -1)
            cv2.drawContours(frame, [box], 0, (255, 255, 255), 2)
            cv2.putText(
                frame,
                f"SIM TARGET ({self._sim_color.upper()})",
                (center_x - 60, center_y - 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1
            )
            cv2.putText(
                frame,
                f"TANKBOT USB-CAM SIM | 640x480 @ {self.target_fps}FPS",
                (15, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 200),
                1
            )
        except Exception:
            # Basic fallback without cv2
            x1 = max(0, center_x - box_size // 2)
            x2 = min(self.width, center_x + box_size // 2)
            y1 = max(0, center_y - box_size // 2)
            y2 = min(self.height, center_y + box_size // 2)
            frame[y1:y2, x1:x2] = bgr

        return frame

    def set_simulation_color(self, color: str):
        """Sets the color of the simulated target object"""
        if color in ["red", "green", "blue"]:
            self._sim_color = color

    def get_frame(self) -> Optional[np.ndarray]:
        """Returns a copy of the latest captured BGR frame"""
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()

    def get_jpeg(self, quality: int = 75) -> Optional[bytes]:
        """Encodes the latest frame as JPEG bytes for streaming"""
        frame = self.get_frame()
        if frame is None:
            return None

        try:
            import cv2
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            ret, buf = cv2.imencode(".jpg", frame, encode_param)
            if ret:
                return buf.tobytes()
        except Exception as e:
            logger.debug(f"JPEG encode error: {e}")

        return None

    def get_fps(self) -> float:
        """Returns real-time measured FPS"""
        return self._fps

    def stop(self):
        """Stops capture thread and releases hardware resources"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        self.is_opened = False
        logger.info("Camera processor stopped.")
