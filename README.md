# Mars Cargo Rover (MARCO)

**By Karley Hammond, Advay Chandra, Samuel Razor, and Katherine Hampton**

## About

This is the code for the third design project for Purdue's Engineering 161 Class, which was to design a small autonomous cargo rover.

## Repository Structure

```
MARCO/
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
│   ├── navigation_system.py# Path planning and navigation
│   ├── sensors.py          # Sensor input abstraction
│   ├── task_manager.py     # Task scheduling
│   └── thermal_system.py   # Temperature monitoring
│
└── poc/                    # Proof of Concept experiments
    ├── poc_example.py      # Navigation POC with PID control
    └── proof_of_concept.py # Basic motor control POC
```

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
- `mobility` - Mobility system development
- `navigation` - Navigation system development  
- `IMU-test` - IMU sensor testing and calibration

## License

See [LICENSE](LICENSE) for details.
