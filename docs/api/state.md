# State API

> Centralized state dataclass shared by all MACRO systems

```python
from systems.state import State
```

---

## State

Dataclass that serves as the single source of truth for all robot state.

### Constructor

```python
state = State()
```

All fields have sensible defaults. The State is typically created once by the Controller and passed to all systems.

---

## Fields

### Position & Motion

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `position` | `np.ndarray` | `[0, 0, 0]` | Robot position [x, y, z] in meters |
| `velocity` | `np.ndarray` | `[0, 0, 0]` | Robot velocity [vx, vy, vz] in m/s |
| `acceleration` | `np.ndarray` | `[0, 0, 0]` | Bias-corrected acceleration in m/s² |
| `acceleration_raw` | `np.ndarray` | `[0, 0, 0]` | Raw acceleration from IMU in m/s² |
| `orientation` | `np.ndarray` | `[0, 0, 0]` | Euler angles [yaw, pitch, roll] in degrees |
| `angular_velocity` | `np.ndarray` | `[0, 0, 0]` | Bias-corrected angular velocity in °/s |
| `angular_velocity_raw` | `np.ndarray` | `[0, 0, 0]` | Raw angular velocity from IMU in °/s |
| `angular_acceleration` | `np.ndarray` | `[0, 0, 0]` | Angular acceleration in °/s² |

### Sensors

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `magnetic_field` | `float` | `0.0` | Magnetic field magnitude in µT |
| `mag_delta` | `float` | `0.0` | Magnetic field difference from baseline in µT |
| `ultrasonic_distance` | `float` | `30.0` | Ultrasonic sensor distance in cm |
| `lf_left_value` | `float` | `0.0` | Left line finder sensor value (0 or 1) |
| `lf_right_value` | `float` | `0.0` | Right line finder sensor value (0 or 1) |
| `color_sensor_value` | `int` | `0` | Color sensor value |
| `button_pressed` | `bool` | `False` | Button state |

### Motors

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `motor_position` | `float` | `0.0` | Front motor position in degrees |
| `motor_velocity` | `float` | `0.0` | Front motor velocity in °/s |
| `turn_position` | `float` | `0.0` | Turn motor position in degrees |

### Cargo

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `cargo_level` | `str` | `"none"` | Detection level: "none", "edge", "semi", "full" |
| `deploying_cargo` | `bool` | `False` | True during cargo deployment (pauses motion) |

### Calibration

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `calibrated_position` | `bool` | `False` | Whether accelerometer is calibrated |
| `calibrated_orientation` | `bool` | `False` | Whether gyroscope is calibrated |
| `calibrated_mag` | `bool` | `False` | Whether magnetometer is calibrated |
| `bias` | `dict` | `None` | Calibration bias values |

### Sensor Positions

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `sensor_positions` | `dict` | `None` | Positions of various sensors relative to ground |

---

## Bias Dictionary

After calibration, `state.bias` contains:

```python
{
    "accel": np.array([ax, ay, az]),  # Accelerometer bias in m/s²
    "gyro": np.array([gx, gy, gz]),   # Gyroscope bias in °/s
    "mag": float                       # Magnetometer baseline in µT
}
```

---

## Usage Examples

### Creating State

```python
from systems.state import State

state = State()
```

### Reading State

```python
# Position
x, y, z = state.position

# Orientation
yaw, pitch, roll = state.orientation

# Sensors
distance = state.ultrasonic_distance
cargo = state.cargo_level
```

### Updating State

State is typically updated by system components:

```python
# SensorInput updates sensor values
state.ultrasonic_distance = sensor.get_distance()
state.magnetic_field = imu.getMag()

# Navigation updates position/orientation
state.position = new_position
state.velocity = new_velocity

# Cargo updates detection
state.cargo_level = "full"
state.deploying_cargo = True
```

### With Controller

```python
from controller import Controller

async def main():
    controller = Controller()
    await controller.initialize()
    
    # Access state through controller
    print(f"Position: {controller.state.position}")
    print(f"Cargo: {controller.state.cargo_level}")
```

---

## System Interactions

### Who Writes What

| Field | Written By |
|-------|-----------|
| `position`, `velocity`, `acceleration` | Navigation |
| `orientation`, `angular_velocity` | Navigation |
| `acceleration_raw`, `angular_velocity_raw` | SensorInput |
| `magnetic_field` | SensorInput |
| `mag_delta`, `cargo_level` | Cargo |
| `deploying_cargo` | Cargo |
| `ultrasonic_distance` | SensorInput |
| `lf_left_value`, `lf_right_value` | SensorInput |
| `motor_position`, `motor_velocity` | MotionController |
| `turn_position` | MotionController |
| `bias`, `calibrated_*` | SensorInput (during calibration) |

### Who Reads What

| System | Reads |
|--------|-------|
| Navigation | `acceleration_raw`, `angular_velocity_raw`, `magnetic_field`, `motor_velocity`, `bias` |
| MotionController | `ultrasonic_distance`, `lf_left_value`, `lf_right_value`, `deploying_cargo` |
| Cargo | `magnetic_field`, `calibrated_mag`, `bias["mag"]` |
| Controller | All fields (for `print_state()`) |

---

## See Also

- [Controller API](controller.md) - How State is created and shared
- [Sensors API](sensors.md) - How sensor values are updated
- [Navigation API](navigation.md) - How position/orientation are calculated
- [Cargo API](cargo.md) - How cargo detection works
