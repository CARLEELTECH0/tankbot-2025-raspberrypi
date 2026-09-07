# 🔌 Hiwonder Tankbot to Raspberry Pi 4B: Male-to-Female Jumper Wiring Guide

This guide walks you through connecting your **Raspberry Pi 4 Model B** directly to the **Hiwonder Tankbot carrier baseboard** (OpenCar4in1) using standard **Male-to-Female jumper wires** after removing the STM32 core board.

---

## 🧭 Socket Orientation on the Tankbot Baseboard

On your STM32 core board (`stm32 hiwonder 1.jpg`), **`Pin1`** is marked at the top corner near the text `STM32单片机`, opposite the reset button (`RST`).

When you remove the STM32 board from the robot, you are left with **two 20-pin female sockets**:
* **Left Socket = `P8`** (Pins 1 to 20, numbered from top to bottom)
* **Right Socket = `P9`** (Pins 1 to 20, numbered from top to bottom)

```text
               TOP (Near Pin 1 Mark)
          ┌───────────────────────────┐
          │  P8 (LEFT)     P9 (RIGHT) │
          │  Socket        Socket     │
          │                           │
  Pin 01  │  [ 1 ]         [ 1 ]      │  Pin 01
  Pin 02  │  [ 2 ]         [ 2 ]      │  Pin 02
  Pin 03  │  [ 3 ]         [ 3 ] ──► GND (Common Ground)
  Pin 04  │  [ 4 ]         [ 4 ]      │  Pin 04
  Pin 05  │  [ 5 ]         [ 5 ] ──► KEY (Button)
  Pin 06  │  [ 6 ]         [ 6 ]      │  Pin 06
  Pin 07  │  [ 7 ]         [ 7 ] ──► I2C SDA
  Pin 08  │  [ 8 ]         [ 8 ] ──► I2C SCL
  Pin 09  │  [ 9 ]         [ 9 ]      │  Pin 09
  Pin 10  │  [ 10]         [ 10]      │  Pin 10
  Pin 11  │  [ 11]         [ 11]      │  Pin 11
  Pin 12  │  [ 12]         [ 12]      │  Pin 12
  Pin 13  │  [ 13] ◄── IN2 [ 13]      │  Pin 13
  Pin 14  │  [ 14] ◄── IN3 [ 14]      │  Pin 14
  Pin 15  │  [ 15] ◄── IN4 [ 15] ──► IN1 (Left Motor)
  Pin 16  │  [ 16] ◄── LED [ 16]      │  Pin 16
  Pin 17  │  [ 17]         [ 17] ──► Ultrasonic Echo
  Pin 18  │  [ 18] ◄── BZ  [ 18] ──► Ultrasonic Trig
  Pin 19  │  [ 19] ◄── TX  [ 19] ──► Servo TX Enable
  Pin 20  │  [ 20] ◄── RX  [ 20] ──► Servo RX Enable
          └───────────────────────────┘
              BOTTOM (Near RST Button)
```

---

## 📋 Complete Wire-by-Wire Connection Table

Insert the **MALE** pin of the jumper into the robot's baseboard socket hole, and the **FEMALE** end onto the Raspberry Pi 4B 40-pin header.

| Signal Function | Baseboard Socket Pin (Male End) | Raspberry Pi 4B Pin (Female End) | Pi 4B BCM GPIO | Wire Color (Suggested) |
| :--- | :--- | :--- | :--- | :--- |
| **Common Ground** | **P9 Pin 3** (`GND`) | **Pin 6** (`GND`) | `GND` | ⬛ Black (Mandatory!) |
| **Left Track IN1** | **P9 Pin 15** (`L298N_IN1`) | **Pin 38** | `GPIO 20` | 🟦 Blue |
| **Left Track IN2** | **P8 Pin 13** (`L298N_IN2`) | **Pin 40** | `GPIO 21` | 🟨 Yellow |
| **Right Track IN3** | **P8 Pin 14** (`L298N_IN3`) | **Pin 37** | `GPIO 26` | 🟩 Green |
| **Right Track IN4** | **P8 Pin 15** (`L298N_IN4`) | **Pin 36** | `GPIO 16` | 🟧 Orange |
| **Bus Servo TX** | **P8 Pin 19** (`Servo_TX`) | **Pin 8** (`UART0 TX`) | `GPIO 14` | 🟪 Purple |
| **Bus Servo RX** | **P8 Pin 20** (`Servo_RX`) | **Pin 10** (`UART0 RX`) | `GPIO 15` | ⬜ White |
| **Bus Servo TX Enable** | **P9 Pin 19** (`Servo_TX_EN`)| **Pin 13** | `GPIO 27` | 🟫 Brown |
| **Bus Servo RX Enable** | **P9 Pin 20** (`Servo_RX_EN`)| **Pin 11** | `GPIO 17` | 🔘 Gray |
| **Ultrasonic Trig** | **P9 Pin 18** (`Trig`) | **Pin 16** | `GPIO 23` | 🟨 Yellow |
| **Ultrasonic Echo** | **P9 Pin 17** (`Echo`) | **Pin 18** *(via 1k/2k divider)*| `GPIO 24` | 🟦 Blue |
| **I2C SDA (Line & IMU)**| **P9 Pin 7** (`SDA`) | **Pin 3** (`I2C1 SDA`) | `GPIO 2` | 🟩 Green |
| **I2C SCL (Line & IMU)**| **P9 Pin 8** (`SCL`) | **Pin 5** (`I2C1 SCL`) | `GPIO 3` | 🟨 Yellow |
| *(Optional) Buzzer* | **P8 Pin 18** (`Buzzer`) | **Pin 7** | `GPIO 4` | 🟧 Orange |
| *(Optional) Status LED*| **P8 Pin 16** (`LED1`) | **Pin 22** | `GPIO 25` | 🟥 Red |
| *(Optional) Push Button*| **P9 Pin 5** (`KEY`) | **Pin 15** | `GPIO 22` | ⬜ White |

---

## ⚡ Power Supply Architecture

> [!IMPORTANT]
> **Do NOT power the Raspberry Pi 4B directly from P9 Pin 4 (`5V`)!**
>
> The Tankbot baseboard uses a small onboard AMS1117-5.0 regulator rated for only ~800mA. The Raspberry Pi 4B requires **5V at 2.5A to 3.0A**.
>
> **Recommended Power Options**:
> 1. **Dual Power (Easiest)**:
>    * Power the robot chassis & servos from the robot's **7.4V LiPo battery** (turned on via the robot power switch).
>    * Power the Raspberry Pi 4B via a standard **5V USB-C power bank** or 5V 3A USB-C battery pack.
>    * **The two systems MUST share Common Ground!** Ensure **P9 Pin 3 (GND)** is connected to **Pi Pin 6 (GND)**.
> 2. **Single Battery (Advanced)**:
>    * Wire a high-efficiency **5V 3A step-down (buck / BEC) converter** from the robot's 7.4V battery switch output to the Raspberry Pi 4B USB-C port or Pins 2/4 (5V) & 6 (GND).

---

## 🛡️ HC-SR04 Ultrasonic 5V Protection Divider

The HC-SR04 Echo pin outputs 5V logic pulses. The Raspberry Pi 4B GPIO pins accept a maximum of 3.3V.

To safely step down the Echo signal:
```text
Baseboard P9 Pin 17 (Echo 5V)
          │
         [1kΩ Resistor]
          │
          ├───► Connect Jumper Wire to Raspberry Pi GPIO 24 (Pin 18)
          │
         [2kΩ Resistor]
          │
Baseboard P9 Pin 3 (GND)
```

*(If you are using an HC-SR04P or RCWL-9610 3.3V-compatible sensor, you can connect directly).*

---

## 🦾 How the Hardware Works Seamlessly

1. **Motors**: The baseboard's L298P H-Bridge has its `Enable A` and `Enable B` pins internally pulled up to 3.3V on the PCB. The Raspberry Pi 4B generates 1 kHz PWM directly on `IN1`, `IN2`, `IN3`, and `IN4`—identical to the original STM32 firmware timer.
2. **Robotic Arm**: The baseboard includes a 74HC126D tri-state buffer. By connecting `Servo_TX`, `Servo_RX`, `Servo_TX_EN`, and `Servo_RX_EN` to the Pi 4B UART0 and GPIOs, the Pi controls the bus servos natively at 115200 baud without needing an external USB servo adapter!
3. **Sensors**: The baseboard's I2C lines (`SDA` and `SCL`) connect directly to the 4-pin expansion headers for the line tracker and MPU6050, allowing the Pi 4B to auto-detect both on I2C bus 1.
