# MACRO Documentation

> Mars Autonomous Cargo Rover Operations - Complete Documentation

**Current Version: 1.0.1**

---

## 📚 Table of Contents

| Document | Description |
|----------|-------------|
| [Getting Started](getting-started.md) | Installation, setup, and first run |
| [Architecture](architecture.md) | System design and module overview |
| [API Reference](api/README.md) | Complete API documentation |
| [Hardware Guide](hardware.md) | Pin configurations and wiring |
| [Testing Guide](testing.md) | Running tests and test modes |
| [Troubleshooting](troubleshooting.md) | Common issues and solutions |

---

## 🚀 Quick Start

```python
import asyncio
from controller import Controller

async def main():
    controller = Controller()  # Loads macro_config.toml
    await controller.initialize()
    await controller.run()

asyncio.run(main())
```

---

## 📁 Project Structure

```
MACRO/
├── main.py                     # Main entry point
├── controller.py               # Central controller with config loading
├── macro_config.toml           # Configuration file for all systems
├── pyproject.toml              # Project configuration (v0.12.4)
├── CHANGELOG.md                # Version history
│
├── basehat/                    # Grove Base HAT sensors
│   ├── imu_sensor.py           # IMU (accelerometer, gyroscope, magnetometer)
│   ├── UltrasonicSensor.py     # Distance sensing
│   ├── line_finder.py          # Line detection
│   ├── hall_sensor.py          # Magnetic position sensing
│   └── button.py               # Button input
│
├── buildhat/                   # Raspberry Pi Build HAT
│   ├── motors.py               # Motor control
│   ├── devices.py              # Device management
│   └── color.py                # Color sensor
│
├── systems/                    # Core systems
│   ├── state.py                # Centralized State dataclass
│   ├── sensors.py              # Sensor abstraction (SensorInput)
│   ├── navigation_system.py    # 3D navigation (Transformation, Location, Navigation)
│   ├── mobility_system.py      # Motor control (MotionController)
│   └── cargo_system.py         # Cargo detection and deployment (Cargo)
│
├── ui/                         # User interface
│   └── navigation_display.py   # Real-time visualization (NavigationDisplay)
│
└── docs/                       # Documentation (you are here)
```

---

## 🔧 Key Features (v0.12.4)

### Controller
- **Centralized Config** - All settings from `macro_config.toml`
- **Shared State** - Single `State` dataclass across all systems
- **Graceful Shutdown** - Proper cleanup on exit
- **Configurable Output** - `print_state(fields=["position", "velocity"])`

### Navigation System
- **3D Position Tracking** - Dead reckoning with IMU integration
- **Motor Encoder Velocity** - Uses motor position for reliable velocity decay
- **Magnetic Field Sensing** - Real-time magnitude with calibration baseline
- **Sensor Position Tracking** - Calculate all sensor positions relative to IMU

### Mobility System
- **Line Following** - Automatic line following with state machine
- **Safety Ring** - Ultrasonic obstacle detection with slowdown/stop zones
- **Motor Encoder Tracking** - Position and velocity from motor encoders
- **Cargo Pause** - Automatically pauses during cargo deployment

### Cargo System
- **Magnetic Detection** - Edge, semi, and full cargo detection levels
- **Auto-Deploy** - Automatically deploys on full cargo detection
- **Debouncing** - Prevents false positives from motor EMF
- **One-Time Deploy** - Deploys once then prevents re-deployment

### Navigation Display
- **Dynamic Scaling** - Auto-adjusts to window size and world bounds
- **Zoom Controls** - Mouse wheel, +/- keys, programmatic zoom
- **Magnetic Indicator** - Ring around rover showing field intensity

---

## 📖 Documentation Index

### Core Modules

| Module | Classes | Description |
|--------|---------|-------------|
| `controller` | `Controller`, `Config` | Central controller with config loading |
| `state` | `State` | Centralized state dataclass |
| `sensors` | `SensorInput` | Hardware sensor abstraction |
| `navigation_system` | `Transformation`, `Location`, `Navigation` | 3D position and orientation tracking |
| `mobility_system` | `MotionController` | Motor control with line following |
| `cargo_system` | `Cargo` | Cargo detection and deployment |
| `navigation_display` | `NavigationDisplay` | Real-time visualization |

### API Reference

- [Navigation System API](api/navigation.md)
- [Mobility System API](api/mobility.md)
- [Cargo System API](api/cargo.md)
- [Display System API](api/display.md)
- [Sensor APIs](api/sensors.md)

### Guides

- [Hardware Setup](hardware.md)
- [Testing Guide](testing.md)
- [Troubleshooting](troubleshooting.md)

---

## 🔗 Quick Links

| Resource | Link |
|----------|------|
| Main README | [../README.md](../README.md) |
| Changelog | [../CHANGELOG.md](../CHANGELOG.md) |
| Configuration | [../macro_config.toml](../macro_config.toml) |
| License | [../LICENSE](../LICENSE) |

---

## 📊 Version History

| Version | Date | Highlights |
|---------|------|------------|
| 0.12.4 | 2025-12-07 | Cargo debouncing, deployment blocking fix, line following improvements |
| 0.12.3 | 2025-12-07 | Graceful shutdown sequence |
| 0.12.2 | 2025-12-07 | LineFinder integration fix |
| 0.12.1 | 2025-12-07 | Line follow fix, turn_position tracking |
| 0.12.0 | 2025-12-07 | Centralized config, CargoConfig, print_state in controller |
| 0.11.0 | 2025-12-07 | State dataclass, sensor position tracking |

See [CHANGELOG.md](../CHANGELOG.md) for complete history.
