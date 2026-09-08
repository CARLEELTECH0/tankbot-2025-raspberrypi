#!/usr/bin/env python3
"""
Tankbot 4-PWM Motor Hardware Diagnostic Tool
Tests each individual PWM pin (IN1, IN2, IN3, IN4) and each motor channel independently.
Use this to identify loose jumper wires, wrong socket holes, or inverted polarities.
"""

import sys
import time
from pathlib import Path

# Add project root to sys.path
BASE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE_DIR))

from config import MOTOR_CONFIG
from hardware.hal import HAL


def print_wiring_reference():
    print("\n" + "=" * 65)
    print(" 🔌 TANKBOT MOTOR WIRING REFERENCE (Pure 4-PWM)")
    print("=" * 65)
    print(f" LEFT MOTOR (M1 - Port P3):")
    print(f"   IN1 -> Pi Pin 38 (GPIO {MOTOR_CONFIG['in1']}) ===> Baseboard Socket P9 Pin 15 (Blue wire)")
    print(f"   IN2 -> Pi Pin 40 (GPIO {MOTOR_CONFIG['in2']}) ===> Baseboard Socket P8 Pin 13 (Yellow wire)")
    print(f"\n RIGHT MOTOR (M2 - Port P4):")
    print(f"   IN3 -> Pi Pin 37 (GPIO {MOTOR_CONFIG['in3']}) ===> Baseboard Socket P8 Pin 14 (Green wire)")
    print(f"   IN4 -> Pi Pin 36 (GPIO {MOTOR_CONFIG['in4']}) ===> Baseboard Socket P8 Pin 15 (Orange wire)")
    print(f"\n COMMON GROUND (Mandatory):")
    print(f"   GND -> Pi Pin 6 (GND)          ===> Baseboard Socket P9 Pin 3 (Black wire)")
    print("=" * 65 + "\n")


def test_single_pin(hal, pin_num: int, label: str, duration: float = 2.0, speed: int = 80):
    print(f"▶ Pulsing {label} (GPIO {pin_num}) at {speed}% speed for {duration}s...")
    pwm = hal.get_pwm(pin_num, MOTOR_CONFIG.get("pwm_freq", 1000))
    pwm.start(0)
    pwm.ChangeDutyCycle(speed)
    time.sleep(duration)
    pwm.ChangeDutyCycle(0)
    print(f"  ✓ Finished {label} pulse.\n")


def test_motor_channel(hal, in_a: int, in_b: int, motor_name: str):
    pwm_a = hal.get_pwm(in_a, MOTOR_CONFIG.get("pwm_freq", 1000))
    pwm_b = hal.get_pwm(in_b, MOTOR_CONFIG.get("pwm_freq", 1000))
    pwm_a.start(0)
    pwm_b.start(0)

    print(f"▶ Testing {motor_name}: Direction A (Pin {in_a}=80%, Pin {in_b}=0%)...")
    pwm_a.ChangeDutyCycle(80)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(2.0)

    print(f"  Braking (0%, 0%)...")
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    time.sleep(0.5)

    print(f"▶ Testing {motor_name}: Direction B (Pin {in_a}=0%, Pin {in_b}=80%)...")
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(80)
    time.sleep(2.0)

    print(f"  Braking (0%, 0%)...")
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    print(f"  ✓ {motor_name} test complete.\n")


def main():
    hal = HAL()
    print_wiring_reference()
    print(f"HAL Backend: {hal.backend} (Simulation Mode: {hal.is_simulation})")

    in1 = MOTOR_CONFIG["in1"]
    in2 = MOTOR_CONFIG["in2"]
    in3 = MOTOR_CONFIG["in3"]
    in4 = MOTOR_CONFIG["in4"]

    if len(sys.argv) > 1 and sys.argv[1] == "--auto":
        print("\n🚀 Running Automated Step-by-Step Diagnostic Sequence...")
        time.sleep(1)
        test_single_pin(hal, in1, "IN1 (Left Motor A)", 2.0)
        time.sleep(0.5)
        test_single_pin(hal, in2, "IN2 (Left Motor B)", 2.0)
        time.sleep(0.5)
        test_single_pin(hal, in3, "IN3 (Right Motor A)", 2.0)
        time.sleep(0.5)
        test_single_pin(hal, in4, "IN4 (Right Motor B)", 2.0)
        hal.cleanup()
        print("Done!")
        return

    while True:
        print("Select Diagnostic Test:")
        print("  1. Pulse IN1 only (GPIO 20 / Pin 38 -> P9 Pin 15) [Left Motor]")
        print("  2. Pulse IN2 only (GPIO 21 / Pin 40 -> P8 Pin 13) [Left Motor]")
        print("  3. Pulse IN3 only (GPIO 26 / Pin 37 -> P8 Pin 14) [Right Motor]")
        print("  4. Pulse IN4 only (GPIO 16 / Pin 36 -> P8 Pin 15) [Right Motor]")
        print("  5. Full Left Motor Cycle (IN1 then IN2)")
        print("  6. Full Right Motor Cycle (IN3 then IN4)")
        print("  7. Run Full Automated Sequential Diagnostic (All 4 Pins)")
        print("  w. Show Wiring Reference Table")
        print("  q. Quit")

        choice = input("Enter choice (1-7/w/q): ").strip().lower()

        try:
            if choice == "1":
                test_single_pin(hal, in1, "IN1 [Left Motor]", 2.5)
            elif choice == "2":
                test_single_pin(hal, in2, "IN2 [Left Motor]", 2.5)
            elif choice == "3":
                test_single_pin(hal, in3, "IN3 [Right Motor]", 2.5)
            elif choice == "4":
                test_single_pin(hal, in4, "IN4 [Right Motor]", 2.5)
            elif choice == "5":
                test_motor_channel(hal, in1, in2, "Left Motor (M1)")
            elif choice == "6":
                test_motor_channel(hal, in3, in4, "Right Motor (M2)")
            elif choice == "7":
                test_single_pin(hal, in1, "IN1 (Left Motor)", 2.0)
                test_single_pin(hal, in2, "IN2 (Left Motor)", 2.0)
                test_single_pin(hal, in3, "IN3 (Right Motor)", 2.0)
                test_single_pin(hal, in4, "IN4 (Right Motor)", 2.0)
            elif choice == "w":
                print_wiring_reference()
            elif choice in ("q", "quit", "exit"):
                break
            else:
                print("Invalid option. Please choose 1-7, w, or q.")
        except KeyboardInterrupt:
            print("\nStopped.")
            break

    hal.cleanup()
    print("Exited cleanly.")


if __name__ == "__main__":
    main()
