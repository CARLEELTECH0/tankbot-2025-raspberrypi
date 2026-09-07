# 🤖 Tankbot 2025: Raspberry Pi 4B Edition & Mobile UI Controller

A high-performance **Raspberry Pi 4 Model B** port and mobile web controller for the **Tankbot 2025 tracked robotic platform with 6-DOF robotic manipulator**.

Converted directly from the original STM32 firmware (`Tankbot-20251122T125555Z-1-001.zip`), this project upgrades the microcontroller firmware into an asynchronous Python edge-robotics stack tailored for the Raspberry Pi 4B's Broadcom BCM2711 SoC, featuring hardware PWM channels, high-speed PL011 UART bus-servo control, and a responsive cyber-robotic **Mobile Touch Web UI**.

---

## ⚡ Raspberry Pi 4B Hardware Optimizations

* **Dual Hardware PWM**: Uses the Pi 4B's native hardware PWM channels (`PWM0` on GPIO 18 and `PWM1` on GPIO 19) for jitter-free, ultra-smooth DC motor speed regulation.
* **Stable High-Speed PL011 UART**: Leverages the Pi 4B's primary hardware UART (`/dev/serial0` on GPIO 14/15) at 115200 baud to drive the Hiwonder / LewanSoul 6-DOF serial bus servos without clock drift.
* **Modern OS Support**: Fully compatible with **Raspberry Pi OS (Debian Bookworm 64-bit & Bullseye)** using `gpiozero` and `lgpio` kernel interfaces (no deprecated `sysfs`).
* **Multi-Client Asynchronous Server**: Powered by `aiohttp` and `websockets` streaming real-time full-duplex telemetry at 20 Hz.

---

## 🚀 Key Features

* **Complete STM32 Hardware Port**:
  * **Dual DC Track Drive (M1 / M2)**: PWM differential steering, variable speed (-100 to +100%), spin turns, and deadzone calibration.
  * **6-DOF Robotic Arm**: Full support for Hiwonder / LewanSoul Serial Bus Servos (LX-16A protocol over UART) and standard PWM servos.
  * **HC-SR04 Ultrasonic Distance Sensor**: High-precision pulse timing with median filtering and collision warning zones.
  * **4-Channel Infrared Line Follower**: Differential steering tracking algorithm.
  * **MPU6050 6-Axis IMU**: Real-time pitch, roll, and rollover/posture detection via I2C Bus 1.
* **Intelligent Autonomous Modes**:
  * `MANUAL`: Direct touch joystick and D-Pad steering.
  * `OBSTACLE_AVOIDANCE`: Ported from STM32 Lesson 2 (`< 290mm` detection trigger, auto-reverse and spin evasive maneuver).
  * `OBJECT_FOLLOW`: Ported from STM32 Lesson 1 (Ultrasonic following; maintains optimal 20–35 cm distance).
  * `LINE_FOLLOW`: Autonomous track navigation using 4-channel IR reflectance.
  * `PICK & PLACE`: Exact 8-step robotic arm grasping routine ported from STM32 `Control.c`.
* **Mobile Touch Web UI**:
  * **Zero App Installation**: Connect from any smartphone (iOS Safari, Android Chrome) via local Wi-Fi / Hotspot.
  * **Proportional Touch Joystick**: Smooth 360° spring-back joystick with deadzone protection and touch-gesture locks (`touch-action: none`).
  * **Ultrasonic Radar HUD**: Live distance readout in cm with dynamic safety color rings (Green / Yellow / Red Alert).
  * **6-DOF Joint Studio**: Live sliders with +/- step buttons and one-touch presets (`Home`, `Pick & Place`, `Open Claw`, `Close Claw`, `Wave`, `Rest`).
  * **Big Red EMERGENCY STOP**: Hardware-level immediate motion halt with tactile haptic vibration feedback.
  * **Seamless Simulation Mode**: Automatically activates a virtual robot on non-Pi systems (laptops, PCs) for instant testing without hardware connected.

---

## 📐 Raspberry Pi 4B Pinout Mapping (BCM)

| Component | Function | BCM Pin | Physical Pin | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Left Motor IN1** | Direction | `GPIO 20` | Pin 38 | Digital Out |
| **Left Motor IN2** | Direction | `GPIO 21` | Pin 40 | Digital Out |
| **Left Motor PWM (ENA)** | Speed | `GPIO 18` | Pin 12 | **Pi 4B Hardware PWM0** |
| **Right Motor IN3** | Direction | `GPIO 26` | Pin 37 | Digital Out |
| **Right Motor IN4** | Direction | `GPIO 16` | Pin 36 | Digital Out |
| **Right Motor PWM (ENB)** | Speed | `GPIO 19` | Pin 35 | **Pi 4B Hardware PWM1** |
| **Ultrasonic TRIG** | Trigger | `GPIO 23` | Pin 16 | 3.3V Output Pulse |
| **Ultrasonic ECHO** | Echo | `GPIO 24` | Pin 18 | **Voltage Divider! (1kΩ / 2kΩ: 5V $\to$ 3.3V)** |
| **Serial Bus Servos TX** | UART TX | `GPIO 14` | Pin 8 | 115200 Baud (`/dev/serial0`) |
| **Serial Bus Servos RX** | UART RX | `GPIO 15` | Pin 10 | 115200 Baud |
| **MPU6050 IMU (SDA)** | I2C Data | `GPIO 2` | Pin 3 | I2C Bus 1 (SDA1) |
| **MPU6050 IMU (SCL)** | I2C Clock | `GPIO 3` | Pin 5 | I2C Bus 1 (SCL1) |
| **Line Tracker (L2, L1, R1, R2)**| IR Sensors | `GPIO 5, 6, 12, 25` | Pins 29, 31, 32, 22 | Active Low Inputs |

> [!WARNING]
> **HC-SR04 ECHO Pin 5V Protection**: The HC-SR04 Echo output is 5V. The Raspberry Pi 4B GPIO pins accept a maximum of 3.3V. Always wire a 1kΩ / 2kΩ resistor voltage divider between the Echo pin and `GPIO 24` to protect your Pi 4B.

---

## 🛠️ Automated Setup on Raspberry Pi 4B

Run the automated configuration script directly on your Pi 4B:

```bash
cd /home/pi/tankbot_raspberrypi
sudo bash setup_pi4b.sh
```

This script automatically:
1. Installs all required dependencies (`python3-gpiozero`, `python3-lgpio`, `python3-serial`, `aiohttp`, `websockets`, `smbus2`).
2. Configures `/boot/firmware/config.txt` to enable I2C and high-stability hardware UART (`dtoverlay=miniuart-bt`).
3. Grants GPIO, serial, and I2C group permissions.
4. Registers and enables the `tankbot.service` systemd service for auto-start on boot.

---

## 🚀 Running the Controller

### Start the Server Manually
```bash
python3 main.py --port 8080
```

Console Output:
```text
=================================================================
      🤖  TANKBOT 2025 - RASPBERRY PI 4B EDITION  🤖
=================================================================
  Local Access:      http://127.0.0.1:8080
  Mobile Network UI: http://192.168.1.105:8080
=================================================================
```

### Connect with Your Smartphone
1. Connect your phone to the same Wi-Fi network (or Raspberry Pi hotspot).
2. Open Safari (iOS) or Chrome (Android) and navigate to: `http://<PI4B_IP>:8080`.
3. Tap **Share > Add to Home Screen** to run Tankbot as a full-screen, responsive native-feeling app!

---

## 📂 Project Structure

```text
tankbot_raspberrypi/
├── config.py                 # Pi 4B Hardware PWM & pin mappings
├── main.py                   # Master launcher & CLI
├── setup_pi4b.sh             # Pi 4B automated system setup script
├── requirements.txt          # Python dependencies
├── core/
│   ├── tankbot.py            # Master robot controller & autonomous state machines
│   └── telemetry.py          # Telemetry data model
├── hardware/
│   ├── hal.py                # Hardware Abstraction Layer (gpiozero / lgpio / simulation)
│   ├── motor_controller.py   # Dual DC track H-Bridge driver with hardware PWM
│   ├── servo_controller.py   # 6-DOF Arm (LX-16A Bus Servos & PWM Servos)
│   ├── ultrasonic.py         # HC-SR04 distance driver with median filter
│   ├── line_follower.py      # 4-channel infrared line follower
│   └── imu_sensor.py         # MPU6050 6-axis I2C driver
├── server/
│   └── web_server.py         # aiohttp async web & WebSocket server
├── web/
│   ├── index.html            # Mobile-first touch controller UI
│   ├── style.css             # Cyber-robotic styling & responsive layout
│   └── app.js                # Virtual joystick, WebSocket client, HUD rendering
└── systemd/
    └── tankbot.service       # systemd autostart unit
```
