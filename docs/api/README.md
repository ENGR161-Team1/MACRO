# API Reference

> Complete documentation for all MACRO classes and methods

---

## Modules

| Module | Description |
|--------|-------------|
| [Controller](controller.md) | Central controller and configuration |
| [State](state.md) | Centralized state dataclass |
| [Navigation](navigation.md) | 3D position and orientation tracking |
| [Mobility](mobility.md) | Motor control, line following, and safety |
| [Cargo](cargo.md) | Cargo detection and deployment |
| [Display](display.md) | Real-time visualization |
| [Sensors](sensors.md) | Hardware sensor interfaces |

---

## Quick Reference

### Controller

```python
from controller import Controller

controller = Controller()  # Loads macro_config.toml
await controller.initialize()
await controller.run()
```

| Class | Purpose |
|-------|---------|
| `Controller` | Central controller that manages all systems |
| `Config` | Complete configuration dataclass |
| `SensorConfig` | Sensor configuration |
| `MobilityConfig` | Mobility configuration |
| `NavigationConfig` | Navigation configuration |
| `CargoConfig` | Cargo configuration |

### State

```python
from systems.state import State
```

| Class | Purpose |
|-------|---------|
| `State` | Centralized state dataclass shared by all systems |

### Navigation System

```python
from systems.navigation_system import Transformation, Location, Navigation
```

| Class | Purpose |
|-------|---------|
| `Transformation` | 3D rotation and translation matrices |
| `Location` | Position tracking with IMU |
| `Navigation` | Full navigation with calibration |

### Mobility System

```python
from systems.mobility_system import MotionController
```

| Class | Purpose |
|-------|---------|
| `MotionController` | Motor control with line following and safety ring |

### Cargo System

```python
from systems.cargo_system import Cargo
```

| Class | Purpose |
|-------|---------|
| `Cargo` | Magnetic cargo detection and auto-deployment |

### Display System

```python
from ui.navigation_display import NavigationDisplay
```

| Class | Purpose |
|-------|---------|
| `NavigationDisplay` | Real-time 2D visualization with zoom |

### Sensors

```python
from basehat import IMUSensor, UltrasonicSensor, LineFinder, HallSensor, Button
from systems.sensors import SensorInput
```

| Class | Purpose |
|-------|---------|
| `SensorInput` | Unified sensor abstraction with update loop |
| `IMUSensor` | 9-axis IMU (accelerometer, gyroscope, magnetometer) |
| `UltrasonicSensor` | Distance measurement |
| `LineFinder` | Line detection |
| `HallSensor` | Magnetic position sensing |
| `Button` | Button input |

---

## Common Patterns

### Controller-Based Usage (Recommended)

```python
import asyncio
from controller import Controller

async def main():
    controller = Controller()
    await controller.initialize()
    await controller.run()

asyncio.run(main())
```

### Manual System Setup

```python
import asyncio
from systems.state import State
from systems.sensors import SensorInput
from systems.mobility_system import MotionController
from systems.cargo_system import Cargo

state = State()

sensors = SensorInput(state=state, imu=True, ultrasonic=True)
motion = MotionController(state=state, front_motor="A", turn_motor="B")
cargo = Cargo(state=state, motor_port="C")

async def main():
    # Start sensor updates
    sensor_task = asyncio.create_task(sensors.run_sensor_update())
    
    # Calibrate
    await sensors.calibrate_imu()
    
    # Run systems
    await asyncio.gather(
        motion.auto_line_follow(),
        cargo.run_cargo_update_loop()
    )

asyncio.run(main())
```

asyncio.run(main())
```

### Navigation with Display

```python
import asyncio
from systems.navigation_system import Navigation
from ui.navigation_display import NavigationDisplay
from basehat import IMUSensor

navigator = Navigation(imu=IMUSensor())
display = NavigationDisplay(navigator=navigator)

async def main():
    await navigator.calibrate()
    await asyncio.gather(
        navigator.run_continuous_update(update_interval=0.1),
        display.run_continuous(update_interval=0.1)
    )

asyncio.run(main())
```

---

## Type Conventions

| Type | Description |
|------|-------------|
| `np.array` | NumPy array for vectors |
| `tuple` | Immutable coordinate tuples |
| `float` | Scalar values (meters, degrees, seconds) |
| `str` | Motor port identifiers ("A", "B", "C", "D") |
| `int` | GPIO pin numbers |

---

## Units

| Measurement | Unit |
|-------------|------|
| Position | meters (m) |
| Velocity | meters/second (m/s) |
| Acceleration | meters/second² (m/s²) |
| Orientation | degrees (°) |
| Angular velocity | degrees/second (°/s) |
| Magnetic field | micro-tesla (µT) |
| Distance | centimeters (cm) |
| Time | seconds (s) |

---

## See Also

- [Getting Started](../getting-started.md)
- [Architecture](../architecture.md)
- [Testing Guide](../testing.md)

---

## Mobility System (v2.1.0)

### MotionController

- `async follow_line()`
  - Main async routine for line following. Starts `track_line` before entering the main loop.
  - Handles obstacle avoidance, override modes, and reverse recovery.

- `async track_line()`
  - Independent async loop for updating `State.line_state` from line sensors.
  - Uses instance variables `self.left_in` and `self.right_in` for sensor state.
