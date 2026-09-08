# 🤖 Tankbot 2025: Raspberry Pi 4B Edition & Mobile UI Controller

A high-performance **Raspberry Pi 4 Model B** port and mobile web controller for the **Tankbot 2025 tracked robotic platform with 6-DOF robotic manipulator**.

Converted directly from the original STM32 firmware (`Tankbot-20251122T125555Z-1-001.zip`), this project upgrades the microcontroller firmware into an asynchronous Python edge-robotics stack tailored for the Raspberry Pi 4B's Broadcom BCM2711 SoC, featuring hardware PWM channels, high-speed PL011 UART bus-servo control, and a responsive cyber-robotic **Mobile Touch Web UI**.

---

## ⚡ Raspberry Pi 4B Hardware Optimizations

* **Pure 4-Channel PWM Motor Control**: Direct 1 kHz PWM control over `IN1`, `IN2`, `IN3`, and `IN4` with **zero enable pins required**—perfectly matching the Hiwonder OpenCar4in1 carrier baseboard where L298P `Enable A` and `Enable B` are hardwired high.
* **Native Hiwonder I2C Line Tracker**: Communicates directly over Raspberry Pi I2C Bus 1 (address `0x78` / `0xF0`, register `0x01`) to poll all 4 infrared reflection sensors simultaneously without consuming discrete GPIOs.
* **Stable High-Speed PL011 UART**: Leverages the Pi 4B's primary hardware UART (`/dev/serial0` on GPIO 14/15) at 115200 baud with 74HC126D hardware buffer control to drive the Hiwonder 6-DOF serial bus servos without clock drift.
* **Modern OS Support**: Fully compatible with **Raspberry Pi OS (Debian Bookworm 64-bit & Bullseye)** using `gpiozero` and `lgpio` kernel interfaces (no deprecated `sysfs`).
* **Multi-Client Asynchronous Server**: Powered by `aiohttp` and `websockets` streaming real-time full-duplex telemetry at 20 Hz.

---

## 🚀 Key Features

* **Complete STM32 Hardware Port**:
  * **Dual DC Track Drive (M1 / M2)**: Pure 4-PWM differential steering, variable speed (-100 to +100%), spin turns, and deadzone calibration.
  * **6-DOF Robotic Arm**: Full support for Hiwonder / LewanSoul Serial Bus Servos (LX-16A protocol over UART) with carrier board buffer gating (`TX_EN` / `RX_EN`).
  * **HC-SR04 Ultrasonic Distance Sensor**: High-precision pulse timing with median filtering and collision warning zones.
  * **Hiwonder 4-Channel I2C Line Follower**: Native I2C register polling and autonomous track navigation.
  * **MPU6050 6-Axis IMU**: Real-time pitch, roll, and rollover/posture detection via I2C Bus 1.
* **Intelligent Autonomous Modes**:
  * `MANUAL`: Direct touch joystick and D-Pad steering.
  * `OBSTACLE_AVOIDANCE`: Ported from STM32 Lesson 2 (`< 290mm` detection trigger, auto-reverse and spin evasive maneuver).
  * `OBJECT_FOLLOW`: Ported from STM32 Lesson 1 (Ultrasonic following; maintains optimal 20–35 cm distance).
  * `LINE_FOLLOW`: Autonomous track navigation using Hiwonder I2C line sensor readings.
  * `PICK & PLACE`: Exact robotic arm grasping routine ported from STM32 `Control.c`.
* **Mobile Touch Web UI**:
  * **Zero App Installation**: Connect from any smartphone (iOS Safari, Android Chrome) via local Wi-Fi / Hotspot.
  * **Proportional Touch Joystick**: Smooth 360° spring-back joystick with deadzone protection and touch-gesture locks (`touch-action: none`).
  * **Ultrasonic Radar HUD**: Live distance readout in cm with dynamic safety color rings (Green / Yellow / Red Alert).
  * **6-DOF Joint Studio**: Live sliders with +/- step buttons and one-touch presets (`Home`, `Pick & Place`, `Open Claw`, `Close Claw`, `Wave`, `Rest`).
  * **Big Red EMERGENCY STOP**: Hardware-level immediate motion halt with tactile haptic vibration feedback.
  * **Seamless Simulation Mode**: Automatically activates a virtual robot on non-Pi systems (laptops, PCs) for instant testing without hardware connected.

---

## 📐 Raspberry Pi 4B Pinout Mapping (BCM)

All connections use standard **Male-to-Female jumper wires** directly from carrier board sockets **P8 (Left)** and **P9 (Right)** to the Raspberry Pi 4B:

| Signal Function | Baseboard Socket Pin (Male End) | Raspberry Pi 4B Pin (Female End) | Pi 4B BCM GPIO | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Common Ground** | **P9 Pin 3** (`GND`) | **Pin 6** (`GND`) | `GND` | **Mandatory common ground** |
| **Left Track IN1** | **P9 Pin 15** (`L298N_IN1`) | **Pin 38** | `GPIO 20` | PWM Direction & Speed |
| **Left Track IN2** | **P8 Pin 13** (`L298N_IN2`) | **Pin 40** | `GPIO 21` | PWM Direction & Speed |
| **Right Track IN3** | **P8 Pin 14** (`L298N_IN3`) | **Pin 37** | `GPIO 26` | PWM Direction & Speed |
| **Right Track IN4** | **P8 Pin 15** (`L298N_IN4`) | **Pin 36** | `GPIO 16` | PWM Direction & Speed |
| **Bus Servo TX** | **P8 Pin 19** (`Servo_TX`) | **Pin 8** (`UART0 TX`) | `GPIO 14` | 115200 Baud |
| **Bus Servo RX** | **P8 Pin 20** (`Servo_RX`) | **Pin 10** (`UART0 RX`) | `GPIO 15` | 115200 Baud |
| **Bus Servo TX Enable** | **P9 Pin 19** (`Servo_TX_EN`)| **Pin 13** | `GPIO 27` | 74HC126 buffer gate |
| **Bus Servo RX Enable** | **P9 Pin 20** (`Servo_RX_EN`)| **Pin 11** | `GPIO 17` | 74HC126 buffer gate |
| **Ultrasonic Trig** | **P9 Pin 18** (`Trig`) | **Pin 16** | `GPIO 23` | 3.3V Output Pulse |
| **Ultrasonic Echo** | **P9 Pin 17** (`Echo`) | **Pin 18** | `GPIO 24` | **Use 1kΩ/2kΩ divider (5V $\to$ 3.3V)** |
| **I2C SDA (Line & IMU)**| **P9 Pin 7** (`SDA`) | **Pin 3** (`I2C1 SDA`) | `GPIO 2` | Line tracker (0x78) & IMU (0x68) |
| **I2C SCL (Line & IMU)**| **P9 Pin 8** (`SCL`) | **Pin 5** (`I2C1 SCL`) | `GPIO 3` | Shared I2C Bus 1 clock |
| *(Optional) Buzzer* | **P8 Pin 18** (`Buzzer`) | **Pin 7** | `GPIO 4` | Active Buzzer |
| *(Optional) Status LED*| **P8 Pin 16** (`LED1`) | **Pin 22** | `GPIO 25` | User Indicator LED |
| *(Optional) Push Button*| **P9 Pin 5** (`KEY`) | **Pin 15** | `GPIO 22` | Onboard Key Button |

> [!WARNING]
> **HC-SR04 ECHO Pin 5V Protection**: The HC-SR04 Echo output is 5V. The Raspberry Pi 4B GPIO pins accept a maximum of 3.3V. Always wire a 1kΩ / 2kΩ resistor voltage divider between the Echo pin and `GPIO 24` to protect your Pi 4B. See [WIRING_GUIDE.md](WIRING_GUIDE.md) for full details.

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
