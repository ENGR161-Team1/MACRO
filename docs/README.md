# MACRO Documentation

> Mars Autonomous Cargo Rover Operations - Complete Documentation

**Current Version: 0.9.0**

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
from basehat import IMUSensor
from systems.navigation_system import Navigation3D
from systems.mobility_system import MotionController

# Initialize hardware
imu = IMUSensor()
motion = MotionController(front_motor="A", turn_motor="B")

# Create navigator with motor-based velocity decay
navigator = Navigation3D(
    imu=imu,
    mode="degrees",
    motion_controller=motion,
    velocity_decay=0.04,
    motor_velocity_threshold=1.0
)

async def main():
    # Calibrate while stationary
    await navigator.calibrate(samples=50)
    
    # Run continuous updates
    await navigator.run_continuous_update(
        update_interval=0.1,
        log_state=True,
        print_state=True
    )

asyncio.run(main())
```

---

## 📁 Project Structure

```
MACRO/
├── main.py                     # Main entry point
├── pyproject.toml              # Project configuration (v0.8.2)
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
│   ├── navigation_system.py    # 3D navigation (Transformation3D, Location3D, Navigation3D)
│   ├── mobility_system.py      # Motor control (MotionController)
│   ├── sensors.py              # Sensor abstraction
│   ├── task_manager.py         # Task scheduling
│   └── thermal_system.py       # Thermal management
│
├── ui/                         # User interface
│   └── navigation_display.py   # Real-time visualization (NavigationDisplay)
│
├── tests/                      # Test files
│   ├── fixtures/               # Shared test configurations
│   ├── navigation_test.py      # Navigation-only testing
│   ├── mobility_test.py        # Mobility with manual/auto modes
│   ├── nav_mobility_test.py    # Combined navigation + mobility
│   └── navigation_display_test.py  # Full visualization test
│
└── docs/                       # Documentation (you are here)
```

---

## 🔧 Key Features (v0.8.2)

### Navigation System
- **3D Position Tracking** - Dead reckoning with IMU integration
- **Motor Encoder Velocity** - Uses motor position for reliable velocity decay
- **Magnetic Field Sensing** - Real-time magnitude with calibration baseline
- **Customizable State Output** - `print_state(fields=["position", "velocity"])`

### Mobility System
- **Safety Ring** - Ultrasonic obstacle detection with slowdown/stop zones
- **Motor Encoder Tracking** - Position and velocity from motor encoders
- **Async Operation** - Non-blocking motor and sensor updates

### Navigation Display
- **Dynamic Scaling** - Auto-adjusts to window size and world bounds
- **Zoom Controls** - Mouse wheel, +/- keys, programmatic zoom
- **Magnetic Indicator** - Ring around rover showing field intensity
- **Resizable Window** - Adjustable world bounds (-10m to 10m default)

---

## 📖 Documentation Index

### Core Modules

| Module | Classes | Description |
|--------|---------|-------------|
| `navigation_system` | `Transformation3D`, `Location3D`, `Navigation3D` | 3D position and orientation tracking |
| `mobility_system` | `MotionController` | Motor control with safety features |
| `navigation_display` | `NavigationDisplay` | Real-time visualization |

### API Reference

- [Navigation System API](api/navigation.md)
- [Mobility System API](api/mobility.md)
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
| Contributing | [../CONTRIBUTING.md](../CONTRIBUTING.md) |
| License | [../LICENSE](../LICENSE) |

---

## 📊 Version History

| Version | Date | Highlights |
|---------|------|------------|
| 0.8.2 | 2024-12-06 | Dynamic zoom, magnetic ring indicator |
| 0.8.1 | 2024-12-06 | Motor velocity display, gradient range fix |
| 0.8.0 | 2024-12-06 | Motor encoder velocity tracking, test fixtures |
| 0.7.1 | 2024-12-06 | Magnetic gradient display background |
| 0.7.0 | 2024-12-06 | Magnetic field sensing, async updates |

See [CHANGELOG.md](../CHANGELOG.md) for complete history.
