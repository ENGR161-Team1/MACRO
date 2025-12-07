# Hardware Guide

> Pin configurations, wiring diagrams, and hardware setup for MARCO

---

## Hardware Overview

### Raspberry Pi 4

The main computing platform running the MARCO system.

```
┌─────────────────────────────────────────┐
│  Raspberry Pi 4                         │
│  ┌─────────────────────────────────────┤
│  │ GPIO Header (40-pin)                 │
│  │ ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● │
│  │ ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● │
│  └─────────────────────────────────────┤
│                                         │
│  [USB] [USB] [ETH] [USB-C]             │
└─────────────────────────────────────────┘
```

---

## HAT Configuration

### Dual HAT Stack

MARCO uses two HATs stacked on the Raspberry Pi:

```
       ┌────────────────────────┐
       │   Grove Base HAT       │ ← Top (Sensors)
       │   ● Ultrasonic         │
       │   ● IMU (I2C)          │
       │   ● Line Finders       │
       ├────────────────────────┤
       │   Build HAT            │ ← Middle (Motors)
       │   [A] [B] [C] [D]      │
       ├────────────────────────┤
       │   Raspberry Pi 4       │ ← Bottom
       └────────────────────────┘
```

### Grove Base HAT

Provides GPIO and I2C connectivity for analog/digital sensors.

| Port | Type | Pin(s) | Typical Use |
|------|------|--------|-------------|
| D5 | Digital | 5 | Line Finder Right |
| D16 | Digital | 16 | Line Finder Left |
| D22 | Digital | 22 | Button |
| D26 | Digital | 26 | Ultrasonic |
| A0 | Analog | A0 | Potentiometer |
| I2C | I2C | SDA/SCL | IMU Sensor |

### Build HAT

Controls LEGO Technic motors and sensors via LPF2 ports.

| Port | Motor Type | Typical Use |
|------|------------|-------------|
| A | Large Motor | Front drive (wheels) |
| B | Large Motor | Turn/steering |
| C | Medium Motor | Auxiliary |
| D | Medium Motor | Payload deployment |

---

## Wiring Diagrams

### Ultrasonic Sensor

```
Grove Base HAT           Ultrasonic Sensor
     D26  ●───────────────● Signal
     VCC  ●───────────────● VCC (5V)
     GND  ●───────────────● GND
```

**Connection:** Grove 4-pin cable to D26 port

### IMU Sensor (9-Axis)

```
Grove Base HAT           IMU Sensor
     SDA  ●───────────────● SDA
     SCL  ●───────────────● SCL
     VCC  ●───────────────● VCC (3.3V)
     GND  ●───────────────● GND
```

**Connection:** Grove I2C cable to any I2C port

**I2C Address:** 0x68 (default)

### Line Finder Sensors

```
Grove Base HAT           Line Finder (Left)
     D16  ●───────────────● Signal
     VCC  ●───────────────● VCC
     GND  ●───────────────● GND

Grove Base HAT           Line Finder (Right)
     D5   ●───────────────● Signal
     VCC  ●───────────────● VCC
     GND  ●───────────────● GND
```

### Motor Connections (Build HAT)

```
Build HAT                LEGO Motors
   Port A ●──[LPF2]───● Front Drive Motor
   Port B ●──[LPF2]───● Turn Motor
   Port C ●──[LPF2]───● Auxiliary Motor
   Port D ●──[LPF2]───● Payload Motor
```

**Cable:** LEGO Powered Up (LPF2) 6-wire cable

---

## GPIO Pin Reference

### Full GPIO Layout

```
   3V3  (1) (2)  5V
 GPIO2  (3) (4)  5V
 GPIO3  (5) (6)  GND
 GPIO4  (7) (8)  GPIO14
   GND  (9) (10) GPIO15
GPIO17 (11) (12) GPIO18
GPIO27 (13) (14) GND
GPIO22 (15) (16) GPIO23
   3V3 (17) (18) GPIO24
GPIO10 (19) (20) GND
 GPIO9 (21) (22) GPIO25
GPIO11 (23) (24) GPIO8
   GND (25) (26) GPIO7
 GPIO0 (27) (28) GPIO1
 GPIO5 (29) (30) GND
 GPIO6 (31) (32) GPIO12
GPIO13 (33) (34) GND
GPIO19 (35) (36) GPIO16
GPIO26 (37) (38) GPIO20
   GND (39) (40) GPIO21
```

### Pin Assignments for MARCO

| GPIO | Physical Pin | Function | Sensor |
|------|-------------|----------|--------|
| GPIO5 | 29 | Digital Input | Line Finder Right |
| GPIO16 | 36 | Digital Input | Line Finder Left |
| GPIO22 | 15 | Digital Input | Button |
| GPIO26 | 37 | Digital Input | Ultrasonic |
| GPIO2 | 3 | I2C SDA | IMU |
| GPIO3 | 5 | I2C SCL | IMU |

---

## Motor Specifications

### LEGO Technic Large Motor

- **Type:** Angular motor with encoder
- **Speed Range:** -100 to 100 (arbitrary units)
- **Encoder Resolution:** 1 degree
- **Stall Detection:** Built-in
- **Typical Use:** Drive wheels, high-torque applications

### LEGO Technic Medium Motor

- **Type:** Angular motor with encoder
- **Speed Range:** -100 to 100 (arbitrary units)
- **Encoder Resolution:** 1 degree
- **Typical Use:** Payload deployment, auxiliary functions

### Motor Direction Convention

```
Positive Speed (+)  →  Forward / Clockwise
Negative Speed (-)  →  Backward / Counter-clockwise
```

---

## Power Requirements

### Power Budget

| Component | Voltage | Current (typical) |
|-----------|---------|-------------------|
| Raspberry Pi 4 | 5V | 1.5A |
| Build HAT | 8V | 0.5A (idle) |
| Motors (×4) | 8V | 0.5A each (max) |
| Grove Sensors | 3.3V/5V | 100mA total |

### Power Sources

1. **Build HAT:** External 8V battery pack (6× AA or LiPo)
2. **Raspberry Pi:** USB-C 5V/3A power supply or Build HAT passthrough

### Battery Pack Connection

```
Battery Pack (8V)
       │
       ▼
┌─────────────────┐
│   Build HAT     │
│   Power Input   │──→ Powers motors
│                 │──→ Provides 5V to Pi
└─────────────────┘
```

---

## I2C Configuration

### Enable I2C

```bash
sudo raspi-config
# Navigate: Interface Options → I2C → Enable
```

### Verify IMU Detection

```bash
i2cdetect -y 1
```

Expected output (IMU at 0x68):
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

---

## Troubleshooting

### Motor Not Responding

1. Check LPF2 cable connection
2. Verify Build HAT has power
3. Check port assignment in code

```python
# Test motor
from buildhat import Motor
motor = Motor("A")
motor.run_for_degrees(90)
```

### IMU Not Detected

1. Check I2C cable connection
2. Verify I2C is enabled: `sudo raspi-config`
3. Scan I2C bus: `i2cdetect -y 1`

### Ultrasonic Reading Errors

1. Check Grove cable connection
2. Verify pin number matches code
3. Ensure clear path in front of sensor

```python
# Test ultrasonic
from basehat import UltrasonicSensor
us = UltrasonicSensor(26)
print(us.get_distance())
```

---

## See Also

- [Getting Started](getting-started.md)
- [Sensor API Reference](api/sensors.md)
- [Architecture Overview](architecture.md)
