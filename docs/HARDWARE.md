# Hardware Setup Guide

## Overview

MACRO (Mars Autonomous Cargo Rover Operations) runs on a Raspberry Pi with two HAT boards for sensor and motor interfacing.

## Required Hardware

### Core Components

| Component | Description |
|-----------|-------------|
| Raspberry Pi 4B | Main processing unit (4GB+ RAM recommended) |
| Grove Base HAT | Sensor interface HAT |
| Raspberry Pi Build HAT | LEGO motor interface HAT |
| LEGO Technic Motors | Drive and steering motors |
| Power Supply | 5V for Pi, 8V for Build HAT motors |

### Sensors

| Sensor | Interface | Pin/Port |
|--------|-----------|----------|
| IMU (MPU9250/ICM20948) | I2C | SDA/SCL |
| Ultrasonic (HC-SR04) | GPIO | D26 (default) |
| Button | GPIO | D5 (default) |
| Hall Effect | GPIO | D16 (default) |
| Line Finder | GPIO | (not configured) |

### Motors

| Motor | Port | Function |
|-------|------|----------|
| Drive Motor | A | Forward/reverse propulsion |
| Turn Motor | B | Steering control |

## Wiring Diagram

```
┌─────────────────────────────────────────┐
│              Raspberry Pi 4              │
│                                          │
│  ┌────────────┐    ┌────────────────┐   │
│  │ Grove Base │    │   Build HAT    │   │
│  │    HAT     │    │                │   │
│  │            │    │   A: Drive     │   │
│  │ I2C: IMU   │    │   B: Turn      │   │
│  │ D26: Ultra │    │                │   │
│  │ D5: Button │    │                │   │
│  │ D16: Hall  │    │                │   │
│  └────────────┘    └────────────────┘   │
└─────────────────────────────────────────┘
```

## HAT Stacking

The Grove Base HAT and Build HAT can be stacked on the Raspberry Pi GPIO header. Ensure:

1. **Build HAT** goes directly on the Pi (requires direct access to GPIO)
2. **Grove Base HAT** can be mounted above or use I2C/GPIO extensions
3. Power connections are properly rated for motor loads

## I2C Configuration

Enable I2C on the Raspberry Pi:

```bash
sudo raspi-config
# Interface Options > I2C > Enable
```

Verify I2C devices:

```bash
i2cdetect -y 1
```

Expected addresses:
- `0x68` or `0x69`: IMU sensor

## Power Considerations

| Component | Voltage | Current |
|-----------|---------|---------|
| Raspberry Pi | 5V | 3A recommended |
| Grove Sensors | 3.3V/5V | ~50mA total |
| Build HAT | 8V | Up to 4A for motors |

⚠️ **Warning**: Do not power motors from the Pi's GPIO. Use the Build HAT's dedicated power input.

## Troubleshooting

### IMU Not Detected

1. Check I2C wiring (SDA to SDA, SCL to SCL)
2. Verify I2C is enabled: `sudo raspi-config`
3. Check for address conflicts: `i2cdetect -y 1`

### Ultrasonic Reading Errors

1. Ensure minimum 2cm distance from objects
2. Check GPIO pin configuration in code
3. Verify 5V power to sensor

### Motors Not Responding

1. Check Build HAT power (8V required)
2. Verify motor port connections (A, B, C, D)
3. Update buildhat firmware if needed

## Software Dependencies

```bash
# System packages
sudo apt-get install python3-pip python3-smbus i2c-tools

# Python packages
pip install numpy buildhat
```

See [README.md](../README.md) for full installation instructions.
