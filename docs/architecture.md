# System Architecture

> Overview of MARCO's modular system design

---

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        MACRO System                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │   Sensors    │    │   Systems    │    │     UI       │       │
│  │   (basehat)  │───▶│  (systems/)  │───▶│    (ui/)     │       │
│  └──────────────┘    └──────────────┘    └──────────────┘       │
│         │                   │                   │                │
│         │                   │                   │                │
│         ▼                   ▼                   ▼                │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │  IMUSensor   │    │ Navigation3D │    │NavigationDisp│       │
│  │  Ultrasonic  │    │MotionControl │    │              │       │
│  │  LineFinder  │    │              │    │              │       │
│  └──────────────┘    └──────────────┘    └──────────────┘       │
│                                                                  │
├─────────────────────────────────────────────────────────────────┤
│                        Hardware Layer                            │
│  ┌──────────────┐    ┌──────────────┐                           │
│  │ Grove Base   │    │  Build HAT   │                           │
│  │    HAT       │    │  (motors)    │                           │
│  └──────────────┘    └──────────────┘                           │
└─────────────────────────────────────────────────────────────────┘
```

---

## Module Hierarchy

### 1. Sensor Layer (`basehat/`)

Low-level hardware interfaces for Grove sensors.

| Module | Class | Purpose |
|--------|-------|---------|
| `imu_sensor.py` | `IMUSensor` | 9-axis IMU (accel, gyro, mag) |
| `UltrasonicSensor.py` | `UltrasonicSensor` | Distance measurement |
| `line_finder.py` | `LineFinder` | Line detection |
| `hall_sensor.py` | `HallSensor` | Magnetic position |
| `button.py` | `Button` | Button input |

### 2. Motor Layer (`buildhat/`)

Raspberry Pi Build HAT interface for LEGO motors.

| Module | Class | Purpose |
|--------|-------|---------|
| `motors.py` | `Motor` | Individual motor control |
| `devices.py` | `Device` | Generic device management |
| `color.py` | `ColorSensor` | Color detection |

### 3. Systems Layer (`systems/`)

Core business logic and algorithms.

| Module | Classes | Purpose |
|--------|---------|---------|
| `navigation_system.py` | `Transformation3D`, `Location3D`, `Navigation3D` | 3D position tracking |
| `mobility_system.py` | `MotionController` | Motor control with safety |
| `sensors.py` | Various | Sensor abstraction |
| `task_manager.py` | `TaskManager` | Task scheduling |
| `thermal_system.py` | `ThermalSystem` | Temperature management |

### 4. UI Layer (`ui/`)

User interface and visualization.

| Module | Class | Purpose |
|--------|-------|---------|
| `navigation_display.py` | `NavigationDisplay` | Real-time 2D visualization |

---

## Data Flow

### Navigation Pipeline

```
IMU Sensor
    │
    ├─── getAccel() ──▶ [ax, ay, az] (m/s²)
    │                        │
    │                        ▼
    │                 ┌─────────────┐
    │                 │ Calibration │◀── accel_bias
    │                 │   Bias      │
    │                 └─────────────┘
    │                        │
    │                        ▼
    │                 ┌─────────────┐
    │                 │  Transform  │◀── orientation (yaw, pitch, roll)
    │                 │ Local→Global│
    │                 └─────────────┘
    │                        │
    │                        ▼
    ├─── getGyro() ──▶ [gx, gy, gz] (°/s)
    │                        │
    │                        ▼
    │                 ┌─────────────┐
    │                 │ Integration │
    │                 │   (dt)      │
    │                 └─────────────┘
    │                        │
    │                        ▼
    │                   orientation
    │                        │
    └─── getMag() ───▶ [mx, my, mz] (µT)
                             │
                             ▼
                      magnetic_magnitude


Motor Encoder
    │
    └─── get_position() ──▶ position (°)
                                │
                                ▼
                         ┌─────────────┐
                         │ Differentiate│
                         │    (dt)      │
                         └─────────────┘
                                │
                                ▼
                          motor_velocity
                                │
                                ▼
                         ┌─────────────┐
                         │ Velocity    │
                         │ Decay Check │
                         └─────────────┘
```

### Motor Control Pipeline

```
User Command / Autonomous Logic
            │
            ▼
    ┌───────────────┐
    │MotionController│
    └───────────────┘
            │
    ┌───────┴───────┐
    │               │
    ▼               ▼
front_motor    turn_motor
    │               │
    ▼               ▼
  start()      run_for_degrees()
    │               │
    └───────┬───────┘
            │
            ▼
    ┌───────────────┐
    │  Safety Ring  │◀── Ultrasonic Sensor
    │   (async)     │
    └───────────────┘
            │
            ▼
    ┌───────────────┐
    │ Speed Control │
    │ stop/slow/run │
    └───────────────┘
```

---

## Class Relationships

### Navigation System

```
Transformation3D
       │
       │ (composition)
       ▼
  Location3D
       │
       │ (inheritance)
       ▼
  Navigation3D ◀───── MotionController (optional)
       │
       │ (composition)
       ▼
  NavigationDisplay
```

### Inheritance Hierarchy

```python
# navigation_system.py

class Transformation3D:
    """3D rotation and translation utilities"""
    pass

class Location3D(Transformation3D):
    """Position tracking with IMU"""
    pass

class Navigation3D(Location3D):
    """Full navigation with logging and magnetic field"""
    pass
```

---

## Async Architecture

MACRO uses Python's `asyncio` for concurrent operations.

### Concurrent Tasks

```python
async def main():
    await asyncio.gather(
        navigator.run_continuous_update(),   # Navigation loop
        motion.start_safety_ring(),          # Safety monitoring
        motion.update_motor_state(dt=0.1),   # Encoder tracking
        display.run_continuous()             # UI updates
    )
```

### Task Timing

| Task | Typical Interval | Purpose |
|------|-----------------|---------|
| Navigation update | 100ms | Position/orientation |
| Motor state update | 100ms | Encoder velocity |
| Safety ring check | 50ms | Obstacle detection |
| Display refresh | 100ms | UI rendering |

---

## Configuration Architecture

### Test Fixtures (`tests/fixtures/`)

Centralized configuration using dataclasses:

```python
@dataclass
class NavigationConfig:
    update_interval: float = 0.1
    velocity_decay: float = 0.04
    accel_threshold: float = 0.05
    motor_velocity_threshold: float = 1.0
    # ...

@dataclass  
class MobilityConfig:
    front_motor: str = "A"
    turn_motor: str = "B"
    ultrasonic_pin: int = 26
    # ...
```

### Factory Functions

```python
def create_navigator(config, motion_controller=None):
    """Create Navigation3D from config."""
    return Navigation3D(
        imu=IMUSensor(),
        velocity_decay=config.velocity_decay,
        motion_controller=motion_controller
    )

def create_motion_controller(config):
    """Create MotionController from config."""
    return MotionController(
        front_motor=config.front_motor,
        turn_motor=config.turn_motor
    )
```

---

## Next Steps

- [API Reference](api/README.md) - Detailed class documentation
- [Hardware Guide](hardware.md) - Physical connections
- [Testing Guide](testing.md) - Running and writing tests
