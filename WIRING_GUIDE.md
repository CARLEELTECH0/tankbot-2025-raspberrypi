# 🔌 Tankbot 2025: Raspberry Pi 4B + Red L298N Module + Hiwonder Carrier Wiring Guide

This guide gives you the **complete, exact wire-by-wire schematic** for connecting your **Raspberry Pi 4 Model B** to:
1. **The Standalone Red L298N Motor Driver Module** (driving the 2 DC track motors)
2. **The Hiwonder Carrier Baseboard** (driving the 6-DOF robotic arm bus servos and sensors)

---

## 🗺️ Visual Raspberry Pi 4B 40-Pin Header Map

```text
                               3V3  (1) (2)  5V
               (I2C1 SDA)  GPIO  2  (3) (4)  5V
               (I2C1 SCL)  GPIO  3  (5) (6)  GND ───────────► Common Ground (Black)
                           GPIO  4  (7) (8)  GPIO 14 (TX) ──► Servo TX (Purple)
                               GND  (9) (10) GPIO 15 (RX) ──► Servo RX (White)
          (Servo RX Enable)GPIO 17 (11) (12) GPIO 18
          (Servo TX Enable)GPIO 27 (13) (14) GND
                           GPIO 22 (15) (16) GPIO 23 ───────► Ultrasonic Trig (Yellow)
                               3V3 (17) (18) GPIO 24 ───────► Ultrasonic Echo (Blue)
                           GPIO 10 (19) (20) GND
                           GPIO  9 (21) (22) GPIO 25
                           GPIO 11 (23) (24) GPIO  8
                               GND (25) (26) GPIO  7
                           GPIO  0 (27) (28) GPIO  1
                           GPIO  5 (29) (30) GND
                           GPIO  6 (31) (32) GPIO 12
                           GPIO 13 (33) (34) GND
                           GPIO 19 (35) (36) GPIO 16 ───────► Right Motor IN4 (Orange)
          (Right Motor IN3)GPIO 26 (37) (38) GPIO 20 ───────► Left Motor IN1 (Blue)
                               GND (39) (40) GPIO 21 ───────► Left Motor IN2 (Yellow)
```

---

## 🚗 Part 1: Red L298N Motor Driver Module Wiring

The red L298N module drives the two continuous track DC motors.

```text
                     ┌───────────────────────────┐
  Left Motor (+) ───►│ OUT1                      │
  Left Motor (-) ───►│ OUT2      [ENA Jumper ON] │◄── Keep Black Jumper on ENA
                     │                           │
  Battery + (7.4V)──►│ 12V/VCC   [ENB Jumper ON] │◄── Keep Black Jumper on ENB
  Common GND ───────►│ GND       IN1 IN2 IN3 IN4 │
  (Leave 5V Empty)──►│ 5V         │   │   │   │  │
                     │            │   │   │   │  │
  Right Motor (+)───►│ OUT3       │   │   │   │  │
  Right Motor (-)───►│ OUT4       │   │   │   │  │
                     └────────────┼───┼───┼───┼──┘
                                  │   │   │   │
  Pi Pin 38 (GPIO 20) ────────────┘   │   │   │   [Left Motor A - Blue]
  Pi Pin 40 (GPIO 21) ────────────────┘   │   │   [Left Motor B - Yellow]
  Pi Pin 37 (GPIO 26) ────────────────────┘   │   [Right Motor A - Green]
  Pi Pin 36 (GPIO 16) ────────────────────────┘   [Right Motor B - Orange]
```

### Pin Table: Red L298N Module to Raspberry Pi 4B
| L298N Board Pin | Connected To | Wire Color | Purpose |
| :--- | :--- | :--- | :--- |
| **`IN1`** | **Pi Pin 38** (`GPIO 20`) | 🟦 Blue | Left Track Motor (Reverse) |
| **`IN2`** | **Pi Pin 40** (`GPIO 21`) | 🟨 Yellow | Left Track Motor (Forward) |
| **`IN3`** | **Pi Pin 37** (`GPIO 26`) | 🟩 Green | Right Track Motor (Forward) |
| **`IN4`** | **Pi Pin 36** (`GPIO 16`) | 🟧 Orange | Right Track Motor (Reverse) |
| **`ENA`** | **Jumper Cap INSTALLED** | ⬛ Jumper | Ties Enable A to 5V (Full PWM Control) |
| **`ENB`** | **Jumper Cap INSTALLED** | ⬛ Jumper | Ties Enable B to 5V (Full PWM Control) |
| **`GND`** | **Pi Pin 39 (or Pin 6)** + **Battery (-)** | ⬛ Black | Mandatory Common Ground Reference |
| **`12V / VCC`** | **7.4V Battery (+)** via Power Switch | 🟥 Red | Motor High-Current Power |
| **`OUT1` & `OUT2`**| **Left Track DC Motor wires** | Terminal | Drives Left Track |
| **`OUT3` & `OUT4`**| **Right Track DC Motor wires** | Terminal | Drives Right Track |

---

### 🦾 Part 2: Hiwonder Carrier Board (Robotic Arm Bus Servos + Buzzer ONLY)

Only the 6 serial bus servos and the onboard buzzer go through the Hiwonder sockets **P8** (Left) and **P9** (Right). All sensors connect directly to the Raspberry Pi.

```text
               TOP (White servo ports & power switch)
          ┌──────────────────────────────────────────────────┐
          │  LEFT SOCKET (P8)          RIGHT SOCKET (P9)     │
          │                                                  │
  Pin 01  │  [  EMPTY  ]               [  EMPTY  ]           │ Pin 01
  Pin 02  │  [  EMPTY  ]               [  EMPTY  ]           │ Pin 02
  Pin 03  │  [  EMPTY  ]               [ USE: GND        ] ◄─┼ Pin 03 (Pi Pin 6 / GND)
  ...     │   ... (Pins 4-17 EMPTY)     ... (Pins 4-18 EMPTY)│ ...
  Pin 18  │  [ USE: Buzzer       ] ◄─┼ [  EMPTY  ]           │ Pin 18 (Pi Pin 7 / GPIO 4)
  Pin 19  │  [ USE: Servo TX     ] ◄─┼ [ USE: Servo TX_EN] ◄─┼ Pin 19 (Pi Pin 8 & Pin 13)
  Pin 20  │  [ USE: Servo RX     ] ◄─┼ [ USE: Servo RX_EN] ◄─┼ Pin 20 (Pi Pin 10 & Pin 11)
          └──────┬──────────────────────────┬────────────────┘
                 │                          │
                 ▼                          ▼
          Left: 18, 19, 20           Right: 3, 19, 20
```

### Pin Table: Hiwonder Sockets to Raspberry Pi 4B
| Hiwonder Socket | Socket Pin Number | Physical Position | Raspberry Pi 4B Pin | Pi BCM GPIO | Wire Color | Function |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **P8 (Left)**  | **Pin 18** | 3rd hole from bottom | **Pin 7**  | `GPIO 4`  | 🟧 Orange | Onboard Buzzer |
| **P8 (Left)**  | **Pin 19** | 2nd hole from bottom | **Pin 8**  | `GPIO 14` (UART TX) | 🟪 Purple | Arm Bus Servo TX |
| **P8 (Left)**  | **Pin 20** | Bottom-most hole     | **Pin 10** | `GPIO 15` (UART RX) | ⬜ White  | Arm Bus Servo RX |
| **P9 (Right)** | **Pin 3**  | 3rd hole from top    | **Pin 6**  | `GND`     | ⬛ Black  | System Common Ground |
| **P9 (Right)** | **Pin 19** | 2nd hole from bottom | **Pin 13** | `GPIO 27` | 🟫 Brown  | 74HC126 Buffer TX Gate |
| **P9 (Right)** | **Pin 20** | Bottom-most hole     | **Pin 11** | `GPIO 17` | 🔘 Gray   | 74HC126 Buffer RX Gate |

---

## 📡 Part 3: Sensors (Connected DIRECTLY to Raspberry Pi 4B)

### 1. HC-SR04 Ultrasonic Sensor
* **Trig** $\rightarrow$ Raspberry Pi **GPIO 23** (Physical **Pin 16**)
* **Echo** $\rightarrow$ Raspberry Pi **GPIO 24** (Physical **Pin 18**)
* **VCC**  $\rightarrow$ Raspberry Pi **5V** (Physical **Pin 2** or **Pin 4**)
* **GND**  $\rightarrow$ Raspberry Pi **GND** (Physical **Pin 14** or **Pin 20**)

### 2. I2C Sensors (Line Follower & MPU6050 Accelerometer/Gyro)
* **SDA**  $\rightarrow$ Raspberry Pi **GPIO 2** (Physical **Pin 3**)
* **SCL**  $\rightarrow$ Raspberry Pi **GPIO 3** (Physical **Pin 5**)
* **VCC**  $\rightarrow$ Raspberry Pi **3.3V** (Physical **Pin 1**) or **5V**
* **GND**  $\rightarrow$ Raspberry Pi **GND** (Physical **Pin 9**)

### 3. Sound Sensor
* **Signal/OUT** $\rightarrow$ Raspberry Pi **GPIO 18** (Physical **Pin 12**)
* **VCC**  $\rightarrow$ Raspberry Pi **3.3V** / **5V**
* **GND**  $\rightarrow$ Raspberry Pi **GND**

---

## ⚡ Part 4: Power & Common Ground Architecture

> [!IMPORTANT]
> **COMMON GROUND RULE**:
> All three components **MUST share a common Ground connection**:
> 1. Raspberry Pi Ground (`Pin 6` and `Pin 39`)
> 2. Red L298N Module Ground (`GND` screw terminal)
> 3. Hiwonder Baseboard Ground (`P9 Pin 3`)
> 4. Battery Negative terminal (`-`)
>
> If grounds are not connected together, the PWM signals cannot complete their circuit and motors will jitter or fail to turn!

```text
 ┌──────────────────────┐         ┌──────────────────────┐
 │  7.4V LiPo Battery   │         │ 5V USB-C Power Bank  │
 │  (Robot Main Power)  │         │ (Pi 4B Clean Power)  │
 └──────────┬───────────┘         └──────────┬───────────┘
            │                                │
      [Power Switch]                         │ (5V 3A USB-C)
       ┌────┴───────────────────────────┐    ▼
       │ (+) 7.4V        (-) Ground     │  ┌──────────────────────┐
       │                                │  │   Raspberry Pi 4B    │
       ▼                                ▼  │                      │
┌──────────────┐                 ┌──────┴──┴┐                     │
│ Red L298N    │                 │ Hiwonder │                     │
│ Module       │                 │ Baseboard│                     │
│              │                 │          │                     │
│ 12V   GND    │                 │ 7.4V GND │◄── COMMON GND ──────┤ Pin 6 / Pin 39 (GND)
└──┬─────┬─────┘                 └───┬───┬──┘                     └──────────────────────┘
   │     │                           │   │
   │     └───────── COMMON GND ──────┘   │
   │                                     │
   ▼                                     ▼
2x DC Track Motors              6x Robotic Arm Bus Servos
```

---

## 🚀 Quick Verification Checklist After Wiring

1. **ENA & ENB Jumpers**: Both black jumper caps are installed on the red L298N board.
2. **Grounds**: A wire links Red L298N `GND` $\rightarrow$ Pi Pin 39 $\rightarrow$ Hiwonder P9 Pin 3.
3. **Power On**: Turn on the robot 7.4V switch. The red L298N power LED and Hiwonder power LED will light up.
4. **Test Motors**:
   Run on your Pi:
   ```bash
   python3 tools/test_motors.py --auto
   ```
