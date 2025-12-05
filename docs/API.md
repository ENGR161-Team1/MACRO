# API Reference

## Systems Module

### navigation_system.py

#### Transformation3D

3D transformation utilities for rotation and translation operations.

```python
from systems import Transformation3D

transformer = Transformation3D(mode="degrees")  # or "radians"
```

**Methods:**

| Method | Description | Parameters |
|--------|-------------|------------|
| `get_rotation_yaw(**kwargs)` | Generate Z-axis rotation matrix | `yaw`, `invert` |
| `get_rotation_pitch(**kwargs)` | Generate Y-axis rotation matrix | `pitch`, `invert` |
| `get_rotation_roll(**kwargs)` | Generate X-axis rotation matrix | `roll`, `invert` |
| `get_rotation(**kwargs)` | Combined rotation matrix (ZYX) | `yaw`, `pitch`, `roll`, `invert` |
| `rotate_vector(**kwargs)` | Apply rotation to a vector | `vector`, `yaw`, `pitch`, `roll`, `invert` |
| `translate_vector(**kwargs)` | Apply translation to a vector | `vector`, `translation` |

#### Location3D

3D position tracking using IMU sensor data with dead reckoning.

```python
from systems import Location3D
from basehat import IMUSensor

imu = IMUSensor()
tracker = Location3D(
    imu=imu,
    position=[0.0, 0.0, 0.0],
    orientation=[0.0, 0.0, 0.0],
    mode="degrees",
    velocity_decay=0.02,      # Drift reduction when stationary
    accel_threshold=0.1       # Noise filter threshold (m/s²)
)
```

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `pos` | np.array | Current position [x, y, z] in meters |
| `velocity` | np.array | Current velocity [vx, vy, vz] in m/s |
| `acceleration` | np.array | Global acceleration (gravity-compensated) |
| `orientation` | np.array | Current orientation [yaw, pitch, roll] |
| `calibrated` | bool | Whether IMU has been calibrated |
| `accel_bias` | np.array | Calibrated acceleration bias |
| `gyro_bias` | np.array | Calibrated gyroscope bias |

**Methods:**

| Method | Description |
|--------|-------------|
| `get_position()` | Get current position as tuple (x, y, z) |
| `calibrate(**kwargs)` | Calibrate IMU bias while stationary |
| `update_orientation(**kwargs)` | Update orientation from gyroscope (dt) |
| `update_position(**kwargs)` | Update position from accelerometer (dt, display) |

**calibrate kwargs:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `samples` | int | 50 | Number of samples to average |
| `delay` | float | 0.02 | Delay between samples in seconds |

---

#### Navigation3D

3D navigation system with logging capabilities. Extends Location3D.

```python
from systems import Navigation3D
from basehat import IMUSensor

imu = IMUSensor()
navigator = Navigation3D(
    imu=imu,
    position=[0.0, 0.0, 0.0],
    orientation=[0.0, 0.0, 0.0],
    mode="degrees",
    velocity_decay=0.02,
    accel_threshold=0.1
)
```

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `log` | list | List of logged navigation state entries |
| `start_time` | float | Timestamp when navigation started |
| *Inherits all attributes from Location3D* | | |

**Methods:**

| Method | Description |
|--------|-------------|
| `log_state(timestamp)` | Log current state (position, velocity, acceleration, orientation) |
| `print_state(timestamp)` | Print current state as formatted tuples |
| `run_continuous_update(**kwargs)` | Continuous update loop with optional logging/printing |

**run_continuous_update kwargs:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_interval` | float | 0.1 | Update interval in seconds |
| `log_state` | bool | True | Whether to log state each iteration |
| `print_state` | bool | False | Whether to print state each iteration |
| `calibrate` | bool | True | Whether to calibrate IMU before starting |
| `calibration_samples` | int | 50 | Number of calibration samples |

---

## UI Module

### navigation_display.py

#### NavigationDisplay

Real-time visualization of rover position and navigation data.

```python
from ui import NavigationDisplay
from systems import Navigation3D
from basehat import IMUSensor

imu = IMUSensor()
navigator = Navigation3D(imu=imu)

display = NavigationDisplay(
    width=800,
    height=800,
    scale=50.0,           # pixels per meter
    navigator=navigator
)

# Blocking run
display.run()

# Or async with Navigation3D
async def main():
    await display.run_continuous(update_interval=0.1)
```

**Constructor kwargs:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `width` | int | 800 | Initial window width in pixels |
| `height` | int | 800 | Initial window height in pixels |
| `scale` | float | 50.0 | Pixels per meter |
| `title` | str | "MACRO Navigation Display" | Window title |
| `navigator` | Navigation3D | None | Navigation3D instance for data |

**Display Features:**
- 2D grid with 1m major gridlines (gray #404040) and 0.1m minor gridlines (gray #202020)
- Black dot for rover position with blue velocity arrow
- Arrow direction and length based on velocity vector
- Info panel showing position, orientation, velocity, acceleration
- Resizable window with adaptive canvas

**Methods:**

| Method | Description |
|--------|-------------|
| `start()` | Initialize display window (non-blocking) |
| `run()` | Start display window (blocking) |
| `run_continuous(**kwargs)` | Async loop with navigator updates |
| `update(**kwargs)` | Update display with new data |
| `update_from_navigator()` | Pull data from navigator instance |
| `process_events()` | Process GUI events (call in main loop) |
| `close()` | Close the display window |

**update kwargs:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `position` | tuple | (x, y, z) position in meters |
| `orientation` | tuple | (yaw, pitch, roll) in degrees |
| `velocity` | tuple | (vx, vy, vz) in m/s |
| `acceleration` | tuple | (ax, ay, az) in m/s² |

---

### mobility_system.py

#### MotionController

Motor control and obstacle avoidance for rover movement.

```python
from systems import MotionController

controller = MotionController(
    front_motor="A",
    turn_motor="B",
    ultrasonic_pin=26,
    slowdown_distance=30.0,
    stopping_distance=15.0
)
```

---

### sensors.py

#### SensorInput

Base class for sensor input abstraction.

```python
from systems import SensorInput

sensor = SensorInput()
```

---

## BaseHat Module

Grove Base HAT sensor interfaces.

```python
from basehat import Button, HallSensor, IMUSensor, UltrasonicSensor
```

### IMUSensor

9-DOF IMU sensor (accelerometer, gyroscope, magnetometer).

```python
imu = IMUSensor()

# Get acceleration (m/s²)
ax, ay, az = imu.getAccel()

# Get angular velocity (degrees/second)
gx, gy, gz = imu.getGyro()

# Get magnetic field (micro-tesla)
mx, my, mz = imu.getMag()
```

### UltrasonicSensor

Ultrasonic distance sensor.

```python
sensor = UltrasonicSensor(pin=26)
distance = sensor.get_distance()  # in cm
```

### Button

Digital button input with hold detection.

```python
button = Button(pin=5)
```

### HallSensor

Hall effect sensor for magnetic field detection.

```python
hall = HallSensor(pin=16)
```

---

## BuildHat Module

Raspberry Pi Build HAT interface for LEGO motors.

```python
from buildhat import Motor, MotorPair, ColorSensor, Hat
```

### Motor

Single motor control.

```python
motor = Motor("A")
motor.run_for_seconds(2, speed=50)
```

### MotorPair

Synchronized dual motor control.

```python
pair = MotorPair("A", "B")
pair.start(speed=50)
```

### ColorSensor

LEGO color/light sensor.

```python
sensor = ColorSensor("C")
color = sensor.get_color()
```
