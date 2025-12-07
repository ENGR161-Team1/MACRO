# Navigation System API

> 3D position and orientation tracking with IMU integration

```python
from systems.navigation_system import Transformation3D, Location3D, Navigation3D
```

---

## Transformation3D

3D rotation and translation matrix utilities.

### Constructor

```python
transformer = Transformation3D(mode="degrees")  # or "radians"
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mode` | str | `"degrees"` | Angle unit: `"degrees"` or `"radians"` |

### Methods

#### `get_rotation_yaw(**kwargs)`

Generate Z-axis rotation matrix.

```python
matrix = transformer.get_rotation_yaw(yaw=45.0, invert=False)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `yaw` | float | 0.0 | Rotation angle around Z-axis |
| `invert` | bool | False | If True, creates inverse (transpose) matrix |

**Returns:** `np.array` (3x3 rotation matrix)

#### `get_rotation_pitch(**kwargs)`

Generate Y-axis rotation matrix.

```python
matrix = transformer.get_rotation_pitch(pitch=30.0, invert=False)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pitch` | float | 0.0 | Rotation angle around Y-axis |
| `invert` | bool | False | If True, creates inverse (transpose) matrix |

**Returns:** `np.array` (3x3 rotation matrix)

#### `get_rotation_roll(**kwargs)`

Generate X-axis rotation matrix.

```python
matrix = transformer.get_rotation_roll(roll=15.0, invert=False)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `roll` | float | 0.0 | Rotation angle around X-axis |
| `invert` | bool | False | If True, creates inverse (transpose) matrix |

**Returns:** `np.array` (3x3 rotation matrix)

#### `get_rotation(**kwargs)`

Generate combined rotation matrix (ZYX order: yaw → pitch → roll).

```python
matrix = transformer.get_rotation(yaw=45.0, pitch=30.0, roll=15.0, invert=True)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `yaw` | float | 0.0 | Rotation around Z-axis |
| `pitch` | float | 0.0 | Rotation around Y-axis |
| `roll` | float | 0.0 | Rotation around X-axis |
| `invert` | bool | False | If True, creates inverse matrix (for local→global) |

**Returns:** `np.array` (3x3 rotation matrix)

#### `rotate_vector(**kwargs)`

Apply rotation to a vector.

```python
rotated = transformer.rotate_vector(
    vector=[1.0, 0.0, 0.0],
    yaw=90.0,
    pitch=0.0,
    roll=0.0,
    invert=True
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vector` | array-like | required | 3D vector [x, y, z] |
| `yaw` | float | 0.0 | Rotation around Z-axis |
| `pitch` | float | 0.0 | Rotation around Y-axis |
| `roll` | float | 0.0 | Rotation around X-axis |
| `invert` | bool | False | If True, uses inverse rotation |

**Returns:** `np.array` (rotated 3D vector)

#### `translate_vector(**kwargs)`

Apply translation to a vector.

```python
translated = transformer.translate_vector(
    vector=[1.0, 2.0, 3.0],
    translation=[0.5, 0.5, 0.0]
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `vector` | array-like | required | 3D vector [x, y, z] |
| `translation` | array-like | required | Translation [dx, dy, dz] |

**Returns:** `np.array` (translated 3D vector)

---

## Location3D

3D position tracking using IMU sensor data with dead reckoning.

**Inherits from:** `Transformation3D`

### Constructor

```python
from basehat import IMUSensor
from systems.mobility_system import MotionController

imu = IMUSensor()
motion = MotionController(front_motor="A", turn_motor="B")

tracker = Location3D(
    imu=imu,
    position=[0.0, 0.0, 0.0],
    orientation=[0.0, 0.0, 0.0],
    mode="degrees",
    velocity_decay=0.04,
    accel_threshold=0.05,
    motor_velocity_threshold=1.0,
    motion_controller=motion
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `imu` | IMUSensor | required | IMU sensor instance |
| `position` | list | `[0,0,0]` | Initial position [x, y, z] in meters |
| `orientation` | list | `[0,0,0]` | Initial orientation [yaw, pitch, roll] |
| `mode` | str | `"degrees"` | Angle unit |
| `velocity_decay` | float | 0.04 | Velocity reduction rate when stationary |
| `accel_threshold` | float | 0.05 | Acceleration noise filter (m/s²) |
| `motor_velocity_threshold` | float | 1.0 | Motor velocity threshold for decay (°/s) |
| `motion_controller` | MotionController | None | Optional motor controller for velocity decay |

### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `pos` | np.array | Current position [x, y, z] in meters |
| `velocity` | np.array | Current velocity [vx, vy, vz] in m/s |
| `acceleration` | np.array | Global acceleration (gravity-compensated) |
| `accel_local` | np.array | Local-frame acceleration |
| `orientation` | np.array | Current orientation [yaw, pitch, roll] |
| `d_orientation` | np.array | Angular velocity [dyaw, dpitch, droll] |
| `calibrated` | bool | Whether IMU has been calibrated |
| `accel_bias` | np.array | Calibrated acceleration bias |
| `gyro_bias` | np.array | Calibrated gyroscope bias |
| `motion_controller` | MotionController | Motor controller reference |

### Methods

#### `get_position()`

Get current position as tuple.

```python
x, y, z = tracker.get_position()
```

**Returns:** `tuple` (x, y, z) in meters

#### `async calibrate(**kwargs)`

Calibrate IMU bias while stationary.

```python
await tracker.calibrate(samples=50, delay=0.02)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `samples` | int | 50 | Number of samples to average |
| `delay` | float | 0.02 | Delay between samples (seconds) |

**Note:** Keep the rover completely stationary during calibration!

#### `async update_orientation(**kwargs)`

Update orientation from gyroscope.

```python
await tracker.update_orientation(dt=0.1)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dt` | float | 0.1 | Time step in seconds |

#### `async update_position(**kwargs)`

Update position from accelerometer.

```python
await tracker.update_position(dt=0.1)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dt` | float | 0.1 | Time step in seconds |

---

## Navigation3D

Full 3D navigation system with logging and magnetic field sensing.

**Inherits from:** `Location3D`

### Constructor

```python
navigator = Navigation3D(
    imu=imu,
    position=[0.0, 0.0, 0.0],
    orientation=[0.0, 0.0, 0.0],
    mode="degrees",
    velocity_decay=0.04,
    accel_threshold=0.05,
    motor_velocity_threshold=1.0,
    motion_controller=motion
)
```

*Same parameters as Location3D*

### Additional Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `log` | list | List of logged navigation state entries |
| `start_time` | float | Timestamp when navigation started |
| `magnetic_field` | np.array | Current magnetic field [x, y, z] in µT |
| `magnetic_magnitude` | float | Current magnetic field magnitude in µT |
| `mag_baseline` | float | Calibrated magnetic field baseline |

### Methods

#### `async get_magnetic_field()`

Read and return magnetic field magnitude.

```python
magnitude = await navigator.get_magnetic_field()
print(f"Magnetic field: {magnitude:.2f} µT")
```

**Returns:** `float` (magnitude in µT)

#### `async update_state(**kwargs)`

Update full navigation state (position, orientation, magnetic field).

```python
await navigator.update_state(dt=0.1)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dt` | float | 0.1 | Time step in seconds |

#### `log_state(timestamp)`

Log current state with timestamp.

```python
navigator.log_state(timestamp=1.5)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `timestamp` | float | Current timestamp in seconds |

**Log entry structure:**
```python
{
    "timestamp": 1.5,
    "position": [x, y, z],
    "velocity": [vx, vy, vz],
    "acceleration": [ax, ay, az],
    "orientation": [yaw, pitch, roll],
    "magnetic_magnitude": 45.2
}
```

#### `print_state(timestamp, fields=None)`

Print current state with customizable fields.

```python
# Print all fields
navigator.print_state(timestamp=1.5)

# Print specific fields
navigator.print_state(timestamp=1.5, fields=["position", "velocity"])
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `timestamp` | float | required | Current timestamp |
| `fields` | list | None | Fields to print (None = all) |

**Available fields:**
- `"all"` - All fields
- `"position"` - Position [x, y, z]
- `"velocity"` - Velocity [vx, vy, vz]
- `"acceleration"` - Acceleration [ax, ay, az]
- `"orientation"` - Orientation [yaw, pitch, roll]
- `"magnetic"` - Magnetic field magnitude

#### `async run_continuous_update(**kwargs)`

Run continuous navigation update loop.

```python
await navigator.run_continuous_update(
    update_interval=0.1,
    log_state=True,
    print_state=True,
    print_fields=["position", "velocity"],
    calibrate=True,
    calibration_samples=50
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_interval` | float | 0.1 | Time between updates (seconds) |
| `log_state` | bool | True | Whether to log state |
| `print_state` | bool | True | Whether to print state |
| `print_fields` | list | None | Fields to print |
| `calibrate` | bool | True | Whether to calibrate on start |
| `calibration_samples` | int | 50 | Calibration samples |

---

## Examples

### Basic Navigation

```python
import asyncio
from basehat import IMUSensor
from systems.navigation_system import Navigation3D

async def main():
    imu = IMUSensor()
    nav = Navigation3D(imu=imu)
    
    await nav.calibrate(samples=50)
    
    for _ in range(100):
        await nav.update_state(dt=0.1)
        nav.print_state(timestamp=nav.start_time)
        await asyncio.sleep(0.1)

asyncio.run(main())
```

### With Motor Velocity Decay

```python
import asyncio
from basehat import IMUSensor
from systems.navigation_system import Navigation3D
from systems.mobility_system import MotionController

async def main():
    imu = IMUSensor()
    motion = MotionController(front_motor="A", turn_motor="B")
    
    nav = Navigation3D(
        imu=imu,
        motion_controller=motion,
        motor_velocity_threshold=1.0
    )
    
    await nav.calibrate()
    
    # Run navigation and motor tracking concurrently
    async def track_motors():
        while True:
            await motion.update_motor_state(dt=0.1)
            await asyncio.sleep(0.1)
    
    await asyncio.gather(
        nav.run_continuous_update(),
        track_motors()
    )

asyncio.run(main())
```

---

## See Also

- [Mobility System API](mobility.md)
- [Display System API](display.md)
- [Architecture](../architecture.md)
