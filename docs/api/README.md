# API Reference

> Complete documentation for all MACRO classes and methods

---

## Modules

| Module | Description |
|--------|-------------|
| [Navigation](navigation.md) | 3D position and orientation tracking |
| [Mobility](mobility.md) | Motor control and safety features |
| [Display](display.md) | Real-time visualization |
| [Sensors](sensors.md) | Hardware sensor interfaces |

---

## Quick Reference

### Navigation System

```python
from systems.navigation_system import Transformation, Location, Navigation
```

| Class | Purpose |
|-------|---------|
| `Transformation` | 3D rotation and translation matrices |
| `Location` | Position tracking with IMU |
| `Navigation` | Full navigation with logging and magnetic field |

### Mobility System

```python
from systems.mobility_system import MotionController
```

| Class | Purpose |
|-------|---------|
| `MotionController` | Motor control with safety ring and encoder tracking |

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
```

| Class | Purpose |
|-------|---------|
| `IMUSensor` | 9-axis IMU (accelerometer, gyroscope, magnetometer) |
| `UltrasonicSensor` | Distance measurement |
| `LineFinder` | Line detection |
| `HallSensor` | Magnetic position sensing |
| `Button` | Button input |

---

## Common Patterns

### Async Navigation Loop

```python
import asyncio
from systems.navigation_system import Navigation
from basehat import IMUSensor

navigator = Navigation(imu=IMUSensor())

async def main():
    await navigator.calibrate(samples=50)
    await navigator.run_continuous_update(
        update_interval=0.1,
        log_state=True,
        print_state=True
    )

asyncio.run(main())
```

### Motor with Safety Ring

```python
import asyncio
from systems.mobility_system import MotionController

motion = MotionController(front_motor="A", turn_motor="B")

async def main():
    motion.start()
    await motion.start_safety_ring()

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
