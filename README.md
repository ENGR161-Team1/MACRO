# Mars Autonomous Cargo Rover Operations (MACRO)

**By Karley Hammond, Advay Chandra, Samuel Razor, and Katherine Hampton**

[![Documentation](https://img.shields.io/badge/docs-latest-red.svg)](docs/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Changelog](https://img.shields.io/badge/changelog-latest-green.svg)](CHANGELOG.md) [![Contributing](https://img.shields.io/badge/contributing-guide-blue.svg)](CONTRIBUTING.md)

## About

This is the code for the third design project for Purdue's Engineering 161 Class, which was to design a small autonomous cargo rover.

## Repository Structure

```
MACRO/
├── main.py                 # Main entry point
├── mobility_test.py        # Mobility system testing
├── pyproject.toml          # Project configuration
├── CHANGELOG.md            # Version history and changes
│
├── basehat/                # Grove Base HAT sensor modules
│   ├── button.py           # Button input handling
│   ├── hall_sensor.py      # Hall effect sensor
│   ├── HallSensor.py       # Hall sensor (alternate implementation)
│   ├── imu_sensor.py       # IMU sensor for orientation
│   ├── line_finder.py      # Line detection sensor
│   └── UltrasonicSensor.py # Ultrasonic distance sensor
│
├── buildhat/               # Raspberry Pi Build HAT interface
│   ├── __init__.py         # Package initialization
│   ├── color.py            # Color sensor support
│   ├── devices.py          # Device management
│   ├── exc.py              # Custom exceptions
│   ├── hat.py              # HAT communication
│   ├── motors.py           # Motor control
│   ├── serinterface.py     # Serial interface
│   └── data/               # Firmware and version data
│
├── systems/                # Core rover systems
│   ├── mobility_system.py  # Movement and motor control
│   ├── navigation_system.py# 3D navigation with IMU integration
│   ├── sensors.py          # Sensor input abstraction
│   ├── task_manager.py     # Task scheduling
│   └── thermal_system.py   # Thermal management
│
├── ui/                     # User interface components
│   └── navigation_display.py # Real-time navigation visualization
│
├── tests/                  # Test files
│   ├── navigation_test.py      # Navigation-only test
│   ├── nav-mobility_test.py    # Navigation + mobility test
│   └── navigation_display_test.py # Navigation + display test
│
├── docs/                   # Documentation
│   ├── README.md           # Documentation index
│   ├── API.md              # API reference
│   └── HARDWARE.md         # Hardware setup guide
│
└── poc/                    # Proof of Concept experiments
    ├── poc_example.py      # Navigation POC with PID control
    └── proof_of_concept.py # Basic motor control POC
```

## Features

### Navigation System (v0.7.0)
- **3D Position Tracking**: Dead reckoning with IMU integration
- **IMU Calibration**: Automatic bias measurement for accelerometer, gyroscope, and magnetometer
- **Magnetic Field Sensing**: Real-time magnetic field magnitude detection
- **Drift Reduction**: Velocity decay and acceleration thresholding
- **State Logging**: Timestamped logs of position, velocity, orientation, and magnetic field

### Mobility System
- **Motor Control**: LEGO Technic motors via Build HAT
- **Safety Ring**: Ultrasonic obstacle detection with slowdown/stop zones
- **Async Operation**: Non-blocking motor and sensor updates

### Navigation Display
- **Real-time Visualization**: 2D grid with rover position and velocity arrow
- **Info Panel**: Position, orientation, velocity, acceleration display
- **Resizable Window**: Adaptive canvas scaling

## Hardware

- **Raspberry Pi** with Grove Base HAT
- **Raspberry Pi Build HAT** for LEGO motors
- **Sensors**: IMU, Ultrasonic, Line Finder, Hall Effect, Button
- **Motors**: LEGO Technic motors via Build HAT

## Getting Started

1. Clone the repository
2. Install dependencies: `pip install -e .`
3. Connect hardware components
4. Run: `python main.py`

## Development

See [CHANGELOG.md](CHANGELOG.md) for version history and recent changes.

### Branches

- `main` - Stable, production-ready code
- `systems` - Unified systems development (mobility, navigation, sensors)
- `testing` - Test files and development
- `documentation` - Documentation updates

## License

See [LICENSE](LICENSE) for details.
