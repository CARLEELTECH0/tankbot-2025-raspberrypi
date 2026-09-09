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

## 🦾 Part 2: Hiwonder Carrier Board (Robotic Arm Bus Servos)

The 6 serial bus servos plug directly into the Hiwonder board's servo ports. The Raspberry Pi controls the arm via the onboard 74HC126 bus buffer using Male-to-Female jumpers into sockets **P8** (Left socket) and **P9** (Right socket).

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
  Pin 05  │  [ 5 ]         [ 5 ]      │  Pin 05
  Pin 06  │  [ 6 ]         [ 6 ]      │  Pin 06
  Pin 07  │  [ 7 ]         [ 7 ] ──► I2C SDA
  Pin 08  │  [ 8 ]         [ 8 ] ──► I2C SCL
  ...     │                           │
  Pin 17  │  [ 17]         [ 17] ──► Ultrasonic Echo
  Pin 18  │  [ 18]         [ 18] ──► Ultrasonic Trig
  Pin 19  │  [ 19] ◄── TX  [ 19] ──► Servo TX Enable
  Pin 20  │  [ 20] ◄── RX  [ 20] ──► Servo RX Enable
          └───────────────────────────┘
              BOTTOM (Near RST Button)
```

### Pin Table: Hiwonder Board to Raspberry Pi 4B (Servos)
| Hiwonder Socket Pin | Raspberry Pi 4B Pin | Pi BCM GPIO | Suggested Color | Signal Function |
| :--- | :--- | :--- | :--- | :--- |
| **P8 Pin 19** (`Servo_TX`) | **Pin 8** (`UART0 TX`) | `GPIO 14` | 🟪 Purple | Arm Bus Servo Serial Data TX |
| **P8 Pin 20** (`Servo_RX`) | **Pin 10** (`UART0 RX`)| `GPIO 15` | ⬜ White | Arm Bus Servo Serial Data RX |
| **P9 Pin 19** (`Servo_TX_EN`)| **Pin 13** | `GPIO 27` | 🟫 Brown | 74HC126 Buffer TX Gate Enable |
| **P9 Pin 20** (`Servo_RX_EN`)| **Pin 11** | `GPIO 17` | 🔘 Gray | 74HC126 Buffer RX Gate Enable |
| **P9 Pin 3** (`GND`) | **Pin 6** (`GND`) | `GND` | ⬛ Black | System Common Ground |

---

## 📡 Part 3: Sensors (Line Follower & Ultrasonic)

### 1. I2C Sensors (4-Channel Line Follower & MPU6050 IMU)
* Plug the 4-channel Line Follower into the 4-pin I2C port on the Hiwonder board (`P11` or `P12`).
* Connect the Pi I2C bus to the Hiwonder board:
| Hiwonder Baseboard | Raspberry Pi 4B Pin | Pi BCM GPIO | Wire Color |
| :--- | :--- | :--- | :--- |
| **P9 Pin 7** (`SDA`) | **Pin 3** (`I2C1 SDA`) | `GPIO 2` | 🟩 Green |
| **P9 Pin 8** (`SCL`) | **Pin 5** (`I2C1 SCL`) | `GPIO 3` | 🟨 Yellow |

### 2. Ultrasonic Sensor (HC-SR04)
* Plug the HC-SR04 into the ultrasonic header `P10` on the Hiwonder board.
| Hiwonder Baseboard | Raspberry Pi 4B Pin | Pi BCM GPIO | Wire Color | Note |
| :--- | :--- | :--- | :--- | :--- |
| **P9 Pin 18** (`Trig`) | **Pin 16** | `GPIO 23` | 🟨 Yellow | Trigger pulse |
| **P9 Pin 17** (`Echo`) | **Pin 18** | `GPIO 24` | 🟦 Blue | Echo (Use 1k/2k divider if 5V) |

### 3. Sound Sensor (Hiwonder Microphone Module)
* Plug the 4-pin sensor cable into socket **`P13`** on the Hiwonder baseboard (labeled `5V GND E1 E2`, in the center below the STM32 socket).
* The sensor signal pin routes internally to **Socket P8 Pin 17 (`E1`)**.
| Hiwonder Baseboard | Raspberry Pi 4B Pin | Pi BCM GPIO | Wire Color | Function |
| :--- | :--- | :--- | :--- | :--- |
| **P8 Pin 17** (`E1`) | **Pin 12** | `GPIO 18` | 🟫 Brown | Sound pulse (High on clap/sound trigger) |

### 4. Onboard Audible Buzzer
* The buzzer is the round black cylinder located on the carrier board, driven by onboard NPN transistor `Q1`.
* It is controlled via **Socket P8 Pin 18 (`Buzzer`)**:
| Hiwonder Baseboard | Raspberry Pi 4B Pin | Pi BCM GPIO | Wire Color | Function |
| :--- | :--- | :--- | :--- | :--- |
| **P8 Pin 18** (`Buzzer`) | **Pin 7** | `GPIO 4` | 🟧 Orange | Buzzer control (High=Beep, Low=Off / PWM tone) |

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
