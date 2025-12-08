# System Architecture

> Overview of MACRO's modular system design

---

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        MACRO System                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│                      ┌──────────────┐                           │
│                      │  Controller  │                           │
│                      │ (controller) │                           │
│                      └──────┬───────┘                           │
│                             │                                    │
│              ┌──────────────┼──────────────┐                    │
│              │              │              │                     │
│              ▼              ▼              ▼                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │    State     │◀───│   Systems    │───▶│   Sensors    │       │
│  │ (state.py)   │    │  (systems/)  │    │  (basehat/)  │       │
│  └──────────────┘    └──────────────┘    └──────────────┘       │
│         ▲                   │                                    │
│         │                   ▼                                    │
│         │            ┌──────────────┐                           │
│         └────────────│   Motors     │                           │
│                      │  (buildhat/) │                           │
│                      └──────────────┘                           │
│                                                                  │
├─────────────────────────────────────────────────────────────────┤
│                        Hardware Layer                            │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │ Grove Base   │    │  Build HAT   │    │ LEGO Motors  │       │
│  │    HAT       │    │  (motors)    │    │ (A, B, C)    │       │
│  └──────────────┘    └──────────────┘    └──────────────┘       │
└─────────────────────────────────────────────────────────────────┘
```

---

## Core Components

### Controller (`controller.py`)

Central orchestrator that:
- Loads configuration from `macro_config.toml`
- Initializes all systems with shared State
- Runs async control loops
- Handles graceful shutdown

```python
controller = Controller()
await controller.initialize()
await controller.run()
```

### State (`systems/state.py`)

Single source of truth for all robot state:

```python
@dataclass
class State:
    position: np.ndarray          # [x, y, z] in meters
    velocity: np.ndarray          # [vx, vy, vz] in m/s
    acceleration: np.ndarray      # [ax, ay, az] in m/s²
    orientation: np.ndarray       # [yaw, pitch, roll] in degrees
    magnetic_field: float         # Magnitude in µT
    mag_delta: float              # Difference from baseline
    cargo_level: str              # "none", "edge", "semi", "full"
    deploying_cargo: bool         # Pauses motion when True
    lf_left_value: float          # Left line finder
    lf_right_value: float         # Right line finder
    ultrasonic_distance: float    # Distance in cm
    motor_position: float         # Front motor degrees
    motor_velocity: float         # Front motor °/s
    turn_position: float          # Turn motor degrees
    # ... calibration flags and bias values
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
| `state.py` | `State` | Centralized state dataclass |
| `sensors.py` | `SensorInput` | Sensor abstraction with update loop |
| `navigation_system.py` | `Transformation`, `Location`, `Navigation` | 3D position tracking |
| `mobility_system.py` | `MotionController` | Motor control, line following, safety |
| `cargo_system.py` | `Cargo` | Cargo detection and deployment |

### 4. UI Layer (`ui/`)

User interface and visualization.

| Module | Class | Purpose |
|--------|-------|---------|
| `navigation_display.py` | `NavigationDisplay` | Real-time 2D visualization |

---

## Data Flow

### Sensor → State → Systems Pipeline

```
┌─────────────┐
│ SensorInput │
│  (sensors)  │
└──────┬──────┘
       │ Updates State directly
       ▼
┌─────────────┐
│    State    │◀─── All systems read from State
└──────┬──────┘
       │
   ┌───┴───┬───────────┬──────────────┐
   ▼       ▼           ▼              ▼
┌──────┐ ┌──────┐ ┌──────────┐ ┌───────────┐
│ Nav  │ │Motion│ │  Cargo   │ │Controller │
│      │ │Ctrl  │ │          │ │(print)    │
└──────┘ └──────┘ └──────────┘ └───────────┘
```

### Line Following State Machine

```
                    ┌─────────┐
                    │ CENTER  │
                    └────┬────┘
         ┌───────────────┼───────────────┐
         │               │               │
    Left sensor     Both clear      Right sensor
    triggers        (maintain)       triggers
         │               │               │
         ▼               │               ▼
    ┌─────────┐          │          ┌─────────┐
    │  RIGHT  │◀─────────┴─────────▶│  LEFT   │
    └─────────┘                     └─────────┘
         │                               │
         │    (opposite sensor from      │
         │     nothing → CENTER)         │
         └───────────────────────────────┘
```

### Cargo Detection Flow

```
Magnetometer
      │
      ▼
┌─────────────┐
│ mag_delta   │ = current - baseline
└──────┬──────┘
       │
       ▼
┌─────────────┐     ┌─────────────────────────┐
│detect_cargo │────▶│ Thresholds:             │
│   _level()  │     │  edge: 400 µT           │
└──────┬──────┘     │  semi: 1000 µT          │
       │            │  full: 3000 µT          │
       ▼            └─────────────────────────┘
┌─────────────┐
│ Track max   │ Record distance at peak reading
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Confirm     │ Detect when mag_delta decreases
│ detection   │ (past the cargo)
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Deploy at   │ Wait until: distance_traveled =
│ distance    │ max_mag_distance + deploy_distance
└─────────────┘
```

---

## Async Architecture

MACRO uses Python's `asyncio` for concurrent operations.

### Controller Run Loop

```python
async def run(self):
    # Start concurrent tasks
    line_follow_task = asyncio.create_task(
        self.mobility.auto_line_follow()
    )
    cargo_monitor_task = asyncio.create_task(
        self.cargo.run_cargo_update_loop()
    )
    
    # Main loop
    while self._running:
        await self.navigator.update_state(dt=update_interval)
        if print_state:
            self.print_state(timestamp, fields)
        await asyncio.sleep(update_interval)
```

### Task Timing

| Task | Interval | Purpose |
|------|----------|---------|
| Sensor update | 50ms | Read all sensors |
| Navigation update | 100ms | Position/orientation |
| Motor state update | 50ms | Encoder velocity |
| Line following | 100ms | Turn adjustments |
| Cargo monitoring | 100ms | Magnetic detection |
| Display refresh | 100ms | UI rendering |

---

## Configuration Architecture

### `macro_config.toml`

All settings in a single TOML file:

```toml
[sensors]
imu = true
ultrasonic = true
line_finders = true

[mobility]
front_motor = "A"
turn_motor = "B"

[mobility.speed]
forward = 20
turn = 20

[mobility.line_follow]
wheel_ratio = 9.0
turn_amount = 20
update_interval = 0.1

[cargo.motor]
port = "C"
speed = 100
deploy_angle = 180

[cargo.detection]
required_consecutive = 5

[navigation.update]
print_state = true
print_fields = ["all"]
```

### Config Dataclasses

```python
@dataclass
class SensorConfig:
    imu: bool = True
    ultrasonic: bool = True
    line_finders: bool = False
    # ... pins and offsets

@dataclass
class MobilityConfig:
    front_motor: str = "A"
    turn_motor: str = "B"
    forward_speed: int = 20
    wheel_ratio: float = 9.0
    turn_amount: int = 20
    line_follow_interval: float = 0.1
    # ... speed and safety settings

@dataclass
class CargoConfig:
    motor_port: str = "C"
    motor_speed: int = 100
    deploy_angle: int = 180
    required_detections: int = 5
    # ... thresholds
```

---

## Next Steps

- [API Reference](api/README.md) - Detailed class documentation
- [Hardware Guide](hardware.md) - Physical connections
- [Testing Guide](testing.md) - Running and writing tests
