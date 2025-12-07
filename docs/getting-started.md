# Getting Started

> Installation, setup, and running your first MACRO program

---

## Prerequisites

### Hardware
- Raspberry Pi 4 (recommended) or Pi 3B+
- Grove Base HAT for Raspberry Pi
- Raspberry Pi Build HAT
- LEGO Technic motors (2-3)
- Grove IMU sensor (6-axis or 9-axis)
- Grove Ultrasonic sensor
- Optional: Line finder sensors, Hall effect sensor, buttons

### Software
- Python 3.10 or higher
- Raspbian OS (Bullseye or newer)

---

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/ENGR161-Team1/MARCO.git
cd MARCO
```

### 2. Install Dependencies

```bash
pip install -e .
```

This installs:
- `numpy` - Matrix operations for 3D transformations

### 3. Verify Hardware Connections

Ensure all sensors and motors are connected before running. See [Hardware Guide](hardware.md) for detailed wiring.

---

## First Run

### Basic Navigation Test

```python
import asyncio
from basehat import IMUSensor
from systems.navigation_system import Navigation3D

imu = IMUSensor()
navigator = Navigation3D(imu=imu, mode="degrees")

async def main():
    # Calibrate IMU (keep rover stationary!)
    await navigator.calibrate(samples=50)
    
    # Run for 10 seconds
    await navigator.run_continuous_update(
        update_interval=0.1,
        print_state=True,
        log_state=True
    )

asyncio.run(main())
```

### With Motor Control

```python
import asyncio
from basehat import IMUSensor
from systems.navigation_system import Navigation3D
from systems.mobility_system import MotionController

imu = IMUSensor()
motion = MotionController(
    front_motor="A",
    turn_motor="B",
    ultrasonic_pin=26
)

navigator = Navigation3D(
    imu=imu,
    mode="degrees",
    motion_controller=motion
)

async def main():
    await navigator.calibrate(samples=50)
    
    # Start motor with safety ring
    motion.start()
    
    await asyncio.gather(
        navigator.run_continuous_update(update_interval=0.1),
        motion.start_safety_ring()
    )

asyncio.run(main())
```

### With Visual Display

```python
import asyncio
from basehat import IMUSensor
from systems.navigation_system import Navigation3D
from ui.navigation_display import NavigationDisplay

imu = IMUSensor()
navigator = Navigation3D(imu=imu, mode="degrees")

display = NavigationDisplay(
    width=800,
    height=800,
    navigator=navigator
)

async def main():
    await navigator.calibrate(samples=50)
    
    await asyncio.gather(
        navigator.run_continuous_update(update_interval=0.1),
        display.run_continuous(update_interval=0.1)
    )

asyncio.run(main())
```

---

## Running Tests

### Using Test Files

```bash
# Navigation only
python tests/navigation_test.py

# Mobility with manual/automatic mode selection
python tests/mobility_test.py

# Navigation + Mobility
python tests/nav_mobility_test.py

# Full display test
python tests/navigation_display_test.py
```

### Test Modes

`mobility_test.py` prompts for mode selection:
```
=== Mobility Test ===
Select mode:
  1. Automatic (safety ring)
  2. Manual control
Mode (1/2): 
```

---

## Configuration

### Navigation Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `velocity_decay` | 0.04 | Velocity reduction rate when stationary |
| `accel_threshold` | 0.05 | Acceleration noise filter (m/s²) |
| `motor_velocity_threshold` | 1.0 | Motor velocity for decay trigger (°/s) |

### Mobility Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `slowdown_distance` | 30.0 | Distance to start slowing (cm) |
| `stopping_distance` | 15.0 | Distance to stop completely (cm) |
| `forward_speed` | 20 | Normal forward speed |
| `forward_speed_slow` | 10 | Reduced speed in slowdown zone |

### Display Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `width` | 800 | Window width (pixels) |
| `height` | 800 | Window height (pixels) |
| `world_min` | -10.0 | Minimum world coordinate (meters) |
| `world_max` | 10.0 | Maximum world coordinate (meters) |

---

## Next Steps

- [Architecture Overview](architecture.md) - Understand the system design
- [API Reference](api/README.md) - Detailed class and method documentation
- [Hardware Guide](hardware.md) - Wiring and pin configurations
- [Testing Guide](testing.md) - Comprehensive testing documentation
