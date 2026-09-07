"""
Hardware Abstraction Layer (HAL) - Optimized for Raspberry Pi 4B
Supports modern Raspberry Pi OS (Debian Bookworm & Bullseye 32/64-bit).
Automatically adapts between gpiozero / lgpio / RPi.GPIO and Simulation Mode.
"""

import sys
import time
import logging
from typing import Optional, Dict

logger = logging.getLogger("Tankbot.HAL")

IS_RASPBERRY_PI = False
GPIO_BACKEND = "MOCK"

# Check for Raspberry Pi 4B platform
try:
    with open("/proc/device-tree/model", "r") as f:
        model = f.read().strip()
        if "Raspberry Pi 4" in model:
            logger.info(f"HAL: Detected Hardware: {model}")
            IS_RASPBERRY_PI = True
        elif "Raspberry Pi" in model:
            logger.info(f"HAL: Detected Raspberry Pi platform: {model}")
            IS_RASPBERRY_PI = True
except Exception:
    pass

# Try importing modern Raspberry Pi 4B libraries (gpiozero / lgpio / RPi.GPIO)
GPIOZERO_AVAILABLE = False
RPI_GPIO_AVAILABLE = False
LGPIO_AVAILABLE = False

try:
    import gpiozero
    from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice
    GPIOZERO_AVAILABLE = True
    GPIO_BACKEND = "GPIOZERO"
    IS_RASPBERRY_PI = True
    logger.info("HAL: Using high-performance gpiozero backend (Pi 4B / Bookworm recommended)")
except ImportError:
    try:
        import RPi.GPIO as RPiGPIO
        RPiGPIO.setwarnings(False)
        RPiGPIO.setmode(RPiGPIO.BCM)
        RPI_GPIO_AVAILABLE = True
        GPIO_BACKEND = "RPI_GPIO"
        IS_RASPBERRY_PI = True
        logger.info("HAL: Using RPi.GPIO backend")
    except (ImportError, RuntimeError):
        try:
            import lgpio
            LGPIO_AVAILABLE = True
            GPIO_BACKEND = "LGPIO"
            IS_RASPBERRY_PI = True
            logger.info("HAL: Using lgpio backend")
        except ImportError:
            GPIO_BACKEND = "MOCK"
            logger.info("HAL: Running in SIMULATION / MOCK mode (No physical Pi GPIO detected)")


class MockPWM:
    """Mock PWM channel for non-Pi environments"""
    def __init__(self, pin: int, freq: int):
        self.pin = pin
        self.freq = freq
        self.duty_cycle = 0.0
        self.is_running = False

    def start(self, duty_cycle: float):
        self.duty_cycle = duty_cycle
        self.is_running = True

    def ChangeDutyCycle(self, duty_cycle: float):
        self.duty_cycle = max(0.0, min(100.0, float(duty_cycle)))

    def stop(self):
        self.is_running = False
        self.duty_cycle = 0.0


class GpioZeroPWMWrapper:
    """Wraps gpiozero PWMOutputDevice with standard ChangeDutyCycle interface"""
    def __init__(self, device: 'PWMOutputDevice'):
        self.device = device
        self.duty_cycle = 0.0

    def start(self, duty_cycle: float):
        self.ChangeDutyCycle(duty_cycle)

    def ChangeDutyCycle(self, duty_cycle: float):
        self.duty_cycle = max(0.0, min(100.0, float(duty_cycle)))
        self.device.value = self.duty_cycle / 100.0

    def stop(self):
        self.device.value = 0.0
        self.duty_cycle = 0.0


class HAL:
    """Centralized Hardware Interface for Raspberry Pi 4B"""
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(HAL, cls).__new__(cls)
            cls._instance._init_hal()
        return cls._instance

    def _init_hal(self):
        self.backend = GPIO_BACKEND
        self.is_simulation = (GPIO_BACKEND == "MOCK")
        self._pins: Dict[int, int] = {}
        self._pwms: Dict[int, any] = {}
        self._gpiozero_devices = {}
        self._lgpio_handle = None

        if self.backend == "LGPIO":
            try:
                import lgpio
                self._lgpio_handle = lgpio.gpiochip_open(0)
            except Exception as e:
                logger.error(f"Failed to open lgpio chip: {e}")
                self.backend = "MOCK"
                self.is_simulation = True

    def setup_output(self, pin: int, initial_high: bool = False):
        if self.is_simulation:
            self._pins[pin] = 1 if initial_high else 0
            return

        if self.backend == "GPIOZERO":
            if pin not in self._gpiozero_devices:
                self._gpiozero_devices[pin] = DigitalOutputDevice(pin, initial_value=initial_high)
            return

        if self.backend == "RPI_GPIO":
            import RPi.GPIO as RPiGPIO
            RPiGPIO.setup(pin, RPiGPIO.OUT, initial=RPiGPIO.HIGH if initial_high else RPiGPIO.LOW)
            return

        if self.backend == "LGPIO" and self._lgpio_handle is not None:
            import lgpio
            lgpio.gpio_claim_output(self._lgpio_handle, pin, 1 if initial_high else 0)

    def setup_input(self, pin: int, pull_up: bool = True):
        if self.is_simulation:
            self._pins[pin] = 1 if pull_up else 0
            return

        if self.backend == "GPIOZERO":
            if pin not in self._gpiozero_devices:
                self._gpiozero_devices[pin] = DigitalInputDevice(pin, pull_up=pull_up)
            return

        if self.backend == "RPI_GPIO":
            import RPi.GPIO as RPiGPIO
            pud = RPiGPIO.PUD_UP if pull_up else RPiGPIO.PUD_DOWN
            RPiGPIO.setup(pin, RPiGPIO.IN, pull_up_down=pud)
            return

        if self.backend == "LGPIO" and self._lgpio_handle is not None:
            import lgpio
            flags = lgpio.SET_PULL_UP if pull_up else lgpio.SET_PULL_DOWN
            lgpio.gpio_claim_input(self._lgpio_handle, pin, flags)

    def write_pin(self, pin: int, value: bool):
        val = 1 if value else 0
        self._pins[pin] = val

        if self.is_simulation:
            return

        if self.backend == "GPIOZERO":
            dev = self._gpiozero_devices.get(pin)
            if dev:
                dev.value = val
            else:
                self.setup_output(pin, initial_high=value)
            return

        if self.backend == "RPI_GPIO":
            import RPi.GPIO as RPiGPIO
            RPiGPIO.output(pin, RPiGPIO.HIGH if value else RPiGPIO.LOW)
            return

        if self.backend == "LGPIO" and self._lgpio_handle is not None:
            import lgpio
            lgpio.gpio_write(self._lgpio_handle, pin, val)

    def read_pin(self, pin: int) -> int:
        if self.is_simulation:
            return self._pins.get(pin, 0)

        if self.backend == "GPIOZERO":
            dev = self._gpiozero_devices.get(pin)
            if dev:
                return 1 if dev.value else 0
            return 0

        if self.backend == "RPI_GPIO":
            import RPi.GPIO as RPiGPIO
            return RPiGPIO.input(pin)

        if self.backend == "LGPIO" and self._lgpio_handle is not None:
            import lgpio
            return lgpio.gpio_read(self._lgpio_handle, pin)

        return 0

    def get_pwm(self, pin: int, frequency: int = 1000):
        if pin in self._pwms:
            return self._pwms[pin]

        if self.is_simulation:
            pwm = MockPWM(pin, frequency)
            self._pwms[pin] = pwm
            return pwm

        if self.backend == "GPIOZERO":
            try:
                device = PWMOutputDevice(pin, frequency=frequency, initial_value=0)
                pwm = GpioZeroPWMWrapper(device)
                self._pwms[pin] = pwm
                return pwm
            except Exception as e:
                logger.warning(f"gpiozero PWM failed ({e}), falling back to mock.")
                pwm = MockPWM(pin, frequency)
                self._pwms[pin] = pwm
                return pwm

        if self.backend == "RPI_GPIO":
            import RPi.GPIO as RPiGPIO
            self.setup_output(pin, initial_high=False)
            pwm = RPiGPIO.PWM(pin, frequency)
            self._pwms[pin] = pwm
            return pwm

        pwm = MockPWM(pin, frequency)
        self._pwms[pin] = pwm
        return pwm

    def cleanup(self):
        for pwm in self._pwms.values():
            try:
                pwm.stop()
            except Exception:
                pass
        self._pwms.clear()

        for dev in self._gpiozero_devices.values():
            try:
                dev.close()
            except Exception:
                pass
        self._gpiozero_devices.clear()

        if self.backend == "LGPIO" and self._lgpio_handle is not None:
            try:
                import lgpio
                lgpio.gpiochip_close(self._lgpio_handle)
            except Exception:
                pass

        if self.backend == "RPI_GPIO":
            try:
                import RPi.GPIO as RPiGPIO
                RPiGPIO.cleanup()
            except Exception:
                pass

        logger.info("HAL: GPIO cleanup completed")
