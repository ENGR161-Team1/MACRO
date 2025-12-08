# Controller API

> Central controller for MACRO that manages configuration and all systems

```python
from controller import Controller
```

---

## Controller

Central orchestrator that reads configuration from `macro_config.toml` and initializes all systems with a shared State.

### Constructor

```python
controller = Controller(config_path=None)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `config_path` | str \| None | `None` | Path to config file. If None, uses `macro_config.toml` in project root |

### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `config` | Config | Parsed configuration dataclass |
| `state` | State | Shared state instance |
| `sensors` | SensorInput | Sensor abstraction |
| `mobility` | MotionController | Motor controller |
| `navigator` | Navigation | Navigation system |
| `cargo` | Cargo | Cargo detection system |

---

## Methods

### `async initialize()`

Initialize all systems based on configuration.

```python
await controller.initialize()
```

**Behavior:**
1. Creates SensorInput with configured sensors
2. Creates MotionController with configured motors
3. Creates Navigation with configured parameters
4. Creates Cargo with configured thresholds
5. Starts sensor update loop
6. Calibrates IMU if enabled
7. Starts motor state update loop

---

### `async run()`

Run the main control loop with line following and cargo monitoring.

```python
await controller.run()
```

**Behavior:**
1. Starts line following task (`auto_line_follow()`)
2. Starts cargo monitoring task (`run_cargo_update_loop()`)
3. Runs navigation update loop
4. Prints state if enabled
5. On KeyboardInterrupt, calls `shutdown()`

---

### `async shutdown()`

Graceful shutdown sequence.

```python
await controller.shutdown()
```

**Behavior:**
1. Straighten the wheels
2. Stop sensor update loop
3. Stop all motors

---

### `print_state(timestamp, fields=None)`

Print the current state with timestamp.

```python
controller.print_state(timestamp=1.5, fields=["position", "cargo"])
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `timestamp` | float | required | Current timestamp in seconds since start |
| `fields` | list | `["all"]` | Fields to print |

**Available Fields:**
- `"position"` - Robot position (x, y, z)
- `"velocity"` - Robot velocity
- `"acceleration"` - Robot acceleration
- `"orientation"` - Robot orientation (yaw, pitch, roll)
- `"magnetic"` - Magnetic field magnitude and delta
- `"cargo"` - Cargo detection level
- `"ultrasonic"` - Ultrasonic distance
- `"line_finder"` - Left and right line finder values
- `"motor"` - Front motor position and velocity
- `"turn"` - Turn motor position
- `"all"` - All fields

---

### `stop()`

Stop all systems immediately (synchronous).

```python
controller.stop()
```

---

## Configuration Dataclasses

### Config

Complete MACRO configuration container.

```python
@dataclass
class Config:
    sensors: SensorConfig
    mobility: MobilityConfig
    navigation: NavigationConfig
    cargo: CargoConfig
    display: DisplayConfig
    testing: TestingConfig
```

### SensorConfig

```python
@dataclass
class SensorConfig:
    imu: bool = True
    ultrasonic: bool = True
    line_finders: bool = False
    button: bool = False
    color_sensor: bool = False
    
    # Pins
    ultrasonic_pin: int = 26
    line_finder_left_pin: int = 16
    line_finder_right_pin: int = 5
    button_pin: int = 22
    
    # Ports
    color_sensor_port: str = "D"
    
    # Sensor offsets (meters)
    imu_height: float = 0.015
    color_sensor_height: float = 0.025
    lf_height: float = 0.025
    lf_offset: float = 0.0225
    imu_to_lf: float = 0.125
    imu_to_color: float = 0.11
    imu_to_cargo: float = 0.24
    
    update_interval: float = 0.05
```

### MobilityConfig

```python
@dataclass
class MobilityConfig:
    front_motor: str = "A"
    turn_motor: str = "B"
    forward_speed: int = 20
    forward_speed_slow: int = 10
    turn_speed: int = 20
    max_turn: int = 100
    slowdown_distance: float = 30.0
    stopping_distance: float = 15.0
```

### NavigationConfig

```python
@dataclass
class NavigationConfig:
    mode: str = "degrees"
    position: List[float] = [0.0, 0.0, 0.0]
    orientation: List[float] = [0.0, 0.0, 0.0]
    calibrate: bool = True
    calibration_samples: int = 50
    calibration_delay: float = 0.02
    velocity_decay: float = 0.04
    accel_threshold: float = 0.05
    motor_velocity_threshold: float = 1.0
    update_interval: float = 0.1
    print_state: bool = False
    print_fields: List[str] = ["all"]
```

### CargoConfig

```python
@dataclass
class CargoConfig:
    motor_port: str = "B"
    deploy_angle: int = 180
    edge_threshold: float = 400.0
    semi_threshold: float = 1000.0
    full_threshold: float = 3000.0
    required_detections: int = 5
```

---

## Usage Examples

### Basic Usage

```python
import asyncio
from controller import Controller

async def main():
    controller = Controller()
    await controller.initialize()
    await controller.run()

asyncio.run(main())
```

### Custom Config Path

```python
controller = Controller(config_path="/path/to/custom_config.toml")
```

### Accessing Components

```python
controller = Controller()
await controller.initialize()

# Access state
print(controller.state.position)

# Access systems
controller.mobility.stop()
await controller.cargo.deploy()
```

### Custom Control Loop

```python
async def custom_run():
    controller = Controller()
    await controller.initialize()
    
    try:
        while True:
            await controller.navigator.update_state(dt=0.1)
            controller.print_state(time.time(), ["position", "cargo"])
            
            if controller.state.cargo_level == "full":
                print("Full cargo detected!")
            
            await asyncio.sleep(0.1)
    finally:
        await controller.shutdown()
```

---

## Configuration File

`macro_config.toml` structure:

```toml
[sensors]
imu = true
ultrasonic = true
line_finders = true
update_interval = 0.05

[sensors.pins]
ultrasonic = 26
line_finder_left = 5
line_finder_right = 16

[mobility]
front_motor = "A"
turn_motor = "B"

[mobility.speed]
forward = 20
turn = 20

[mobility.safety]
slowdown_distance = 30.0
stopping_distance = 15.0

[navigation]
mode = "degrees"

[navigation.calibration]
enabled = true
samples = 50

[navigation.update]
print_state = true
print_fields = ["all"]

[cargo.motor]
port = "C"
speed = 100
deploy_angle = 180

[cargo.thresholds]
edge = 400.0
semi = 1000.0
full = 3000.0

[cargo.detection]
required_consecutive = 5
```

---

## See Also

- [State API](state.md) - Shared state dataclass
- [Architecture](../architecture.md) - System overview
- [Getting Started](../getting-started.md) - Setup guide
