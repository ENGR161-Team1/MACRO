# Mars Autonomous Cargo Rover Operations (MACRO)

**By Karley Hammond, Advay Chandra, Samuel Razor, and Katherine Hampton**

[![Documentation](https://img.shields.io/badge/docs-latest-red.svg)](docs/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Changelog](https://img.shields.io/badge/changelog-latest-green.svg)](CHANGELOG.md) [![Contributing](https://img.shields.io/badge/contributing-guide-blue.svg)](CONTRIBUTING.md)

## About

This is the code for the third design project for Purdue's Engineering 161 Class, which was to design a small autonomous cargo rover.

## Repository Structure

```
MACRO/
├── main.py                 # Main entry point
├── controller.py           # Central controller with config loading
├── macro_config.toml       # Configuration file for all systems
├── pyproject.toml          # Project configuration
├── CHANGELOG.md            # Version history and changes
│
├── basehat/                # Grove Base HAT sensor modules
│   ├── button.py           # Button input handling
│   ├── hall_sensor.py      # Hall effect sensor
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
│   ├── state.py            # Centralized State dataclass
│   ├── sensors.py          # Sensor input abstraction
│   ├── mobility_system.py  # Movement, line following, and motor control
│   ├── navigation_system.py# 3D navigation with IMU integration
│   ├── cargo_system.py     # Cargo detection and deployment
│   ├── task_manager.py     # Task scheduling
│   └── thermal_system.py   # Thermal management
│
├── ui/                     # User interface components
│   └── navigation_display.py # Real-time navigation visualization
│
├── tests/                  # Test files
│   └── ...                 # Various test configurations
│
├── docs/                   # Documentation
│   ├── README.md           # Documentation index
│   ├── architecture.md     # System architecture
│   ├── getting-started.md  # Setup guide
│   ├── hardware.md         # Hardware setup guide
│   └── api/                # API reference
│
└── poc/                    # Proof of Concept experiments
    ├── poc_example.py      # Navigation POC with PID control
    └── proof_of_concept.py # Basic motor control POC
```

## Features

### Controller (v2.1.0)
- **Centralized Configuration**: All settings from `macro_config.toml`
- **Shared State**: Single `State` dataclass across all systems
- **Graceful Shutdown**: Proper cleanup on exit (straighten wheels, stop motors)
- **Configurable State Display**: Print selected fields to console
- **Button Mobility Toggle**: Press button to enable/disable robot movement
- **Run Modes**: Display mode (print state) or Control mode (interactive console)
- **Override System**: Trigger straight/left/right overrides via keyboard (w/a/d)
- **Reverse Recovery**: Automatic reverse when stuck in left/right state for too long

### Navigation System
- **3D Position Tracking**: Dead reckoning with IMU integration
- **IMU Calibration**: Automatic bias measurement for accelerometer, gyroscope, and magnetometer
- **Magnetic Field Sensing**: Real-time magnetic field magnitude detection
- **Drift Reduction**: Velocity decay and acceleration thresholding
- **Sensor Position Tracking**: Calculate positions of all sensors relative to IMU

### Mobility System
- **Motor Control**: LEGO Technic motors via Build HAT
- **Line Following**: `follow_line` async routine starts `track_line` for independent line state tracking
- **track_line**: Async loop for updating line state from sensors using instance variables `self.left_in` and `self.right_in`
- **Safety Ring**: Ultrasonic obstacle detection with slowdown/stop zones
- **Async Operation**: Non-blocking motor and sensor updates
- **Cargo Pause**: Automatically pauses during cargo deployment

### Cargo System (v1.1.0)
- **Magnetic Cargo Detection**: Edge, semi, and full detection levels
- **Distance-Based Deployment**: Deploys when robot travels past cargo by configured distance
- **Auto-Deploy**: Automatically deploys on cargo detection confirmation
- **Bay Confirmation**: Confirms cargo bay is open before closing
- **One-Time Deployment**: Deploys once then prevents re-deployment

### Navigation Display
- **Real-time Visualization**: 2D grid with rover position and velocity arrow
- **Info Panel**: Position, orientation, velocity, acceleration display
- **Resizable Window**: Adaptive canvas scaling

## Hardware

- **Raspberry Pi** with Grove Base HAT
- **Raspberry Pi Build HAT** for LEGO motors
- **Sensors**: IMU, Ultrasonic, Line Finder, Hall Effect, Button
- **Motors**: LEGO Technic motors via Build HAT (front, turn, cargo)

## Getting Started

1. Clone the repository
2. Install dependencies: `pip install -e .`
3. Configure `macro_config.toml` for your hardware setup
4. Connect hardware components
5. Run: `python main.py`

## Configuration

All settings are in `macro_config.toml`:

```toml
[sensors]
imu = true
ultrasonic = true
line_finders = true

[mobility]
front_motor = "A"
turn_motor = "B"

[cargo.motor]
port = "C"
speed = 100
deploy_angle = 180

[navigation.update]
print_state = true
print_fields = ["all"]
```

## Development

See [CHANGELOG.md](CHANGELOG.md) for version history and recent changes.

### Branches

- `main` - Stable, production-ready code

## License

See [LICENSE](LICENSE) for details.
