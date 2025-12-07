# Sensor APIs

> Hardware sensor interfaces for Grove Base HAT and Build HAT

```python
from basehat import IMUSensor, UltrasonicSensor, LineFinder, Button
from systems.sensors import SensorInput
```

---

## SensorInput

Centralized sensor management class providing async access to all sensors.

### Constructor

```python
from systems.sensors import SensorInput

sensors = SensorInput(
    ultrasonic_pin=26,
    button_pin=22,
    line_left_pin=16,
    line_right_pin=5,
    color_sensor_port="D"
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ultrasonic_pin` | int | 26 | GPIO pin for ultrasonic sensor |
| `button_pin` | int | 22 | GPIO pin for button |
| `line_left_pin` | int | 16 | GPIO pin for left line finder |
| `line_right_pin` | int | 5 | GPIO pin for right line finder |
| `color_sensor_port` | str | "D" | Build HAT port for color sensor |

### Async Methods

All sensor methods are async and must be awaited.

#### `async get_accel()`

Read acceleration from IMU.

```python
ax, ay, az = await sensors.get_accel()
```

**Returns:** `tuple` (ax, ay, az) in m/s²

#### `async get_gyro()`

Read angular velocity from IMU.

```python
gx, gy, gz = await sensors.get_gyro()
```

**Returns:** `tuple` (gx, gy, gz) in degrees/second

#### `async get_mag()`

Read magnetic field from IMU.

```python
mx, my, mz = await sensors.get_mag()
```

**Returns:** `tuple` (mx, my, mz) in micro-tesla

#### `async get_magnetic_magnitude()`

Get total magnetic field strength.

```python
magnitude = await sensors.get_magnetic_magnitude()
```

**Returns:** `float` (magnitude in µT)

#### `async get_distance()`

Read distance from ultrasonic sensor.

```python
distance = await sensors.get_distance()
```

**Returns:** `float` (distance in cm)

#### `async is_button_pressed()`

Check button state.

```python
pressed = await sensors.is_button_pressed()
```

**Returns:** `bool`

#### `async get_color()`

Get detected color from color sensor.

```python
color = await sensors.get_color()
```

**Returns:** `str` (color name: "black", "white", "red", etc.)

#### `async is_black()`

Check if color sensor detects black.

```python
is_black = await sensors.is_black()
```

**Returns:** `int` (1 for black, 0 for other colors)

#### `async get_line_left()` / `async get_line_right()`

Read line finder values.

```python
left = await sensors.get_line_left()
right = await sensors.get_line_right()
```

**Returns:** `bool` (True if line detected)

### Sensor Availability Methods

```python
sensors.has_imu()         # True if IMU available
sensors.has_ultrasonic()  # True if ultrasonic available
sensors.has_button()      # True if button available
sensors.has_line_left()   # True if left line finder available
sensors.has_line_right()  # True if right line finder available
sensors.has_color()       # True if color sensor available
```

---

## IMUSensor

9-axis IMU with accelerometer, gyroscope, and magnetometer.

### Constructor

```python
from basehat import IMUSensor

imu = IMUSensor()
```

### Methods

#### `getAccel()`

Read acceleration in m/s².

```python
ax, ay, az = imu.getAccel()
print(f"Acceleration: ({ax:.2f}, {ay:.2f}, {az:.2f}) m/s²")
```

**Returns:** `tuple` (ax, ay, az) in m/s²

#### `getGyro()`

Read angular velocity in °/s.

```python
gx, gy, gz = imu.getGyro()
print(f"Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}) °/s")
```

**Returns:** `tuple` (gx, gy, gz) in degrees/second

#### `getMag()`

Read magnetic field in µT.

```python
mx, my, mz = imu.getMag()
print(f"Magnetic: ({mx:.2f}, {my:.2f}, {mz:.2f}) µT")
```

**Returns:** `tuple` (mx, my, mz) in micro-tesla

---

## UltrasonicSensor

Distance measurement using ultrasonic waves.

### Constructor

```python
from basehat import UltrasonicSensor

ultrasonic = UltrasonicSensor(pin=26)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pin` | int | GPIO pin number |

### Methods

#### `get_distance()`

Read distance in centimeters.

```python
distance = ultrasonic.get_distance()
print(f"Distance: {distance:.1f} cm")
```

**Returns:** `float` (distance in cm)

---

## LineFinder

Line detection sensor for line following.

### Constructor

```python
from basehat import LineFinder

line_left = LineFinder(pin=16)
line_right = LineFinder(pin=5)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pin` | int | GPIO pin number |

### Properties

#### `value`

Current line detection state.

```python
if line_left.value:
    print("Line detected on left")
```

**Returns:** `bool` (True if line detected)

---

## HallSensor (Deprecated)

> ⚠️ **Deprecated in v0.9.0**: Use `SensorInput.get_mag()` and `SensorInput.get_magnetic_magnitude()` instead.

Hall effect magnetic position sensor. This sensor has been deprecated in favor of the IMU magnetometer which provides more accurate 3-axis magnetic field readings.

### Migration Guide

```python
# Old (deprecated)
from basehat import HallSensor
hall = HallSensor(pin=12)
value = hall.read()

# New (recommended)
from systems.sensors import SensorInput
sensors = SensorInput()
mx, my, mz = await sensors.get_mag()
magnitude = await sensors.get_magnetic_magnitude()
```

---

## Button

Button input sensor.

### Constructor

```python
from basehat import Button

button = Button(pin=22)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pin` | int | GPIO pin number |

### Properties

#### `is_pressed`

Current button state.

```python
if button.is_pressed:
    print("Button pressed!")
```

**Returns:** `bool` (True if pressed)

---

## Build HAT Motors

LEGO Technic motors via Raspberry Pi Build HAT.

### Constructor

```python
from buildhat import Motor

motor = Motor("A")  # Port A, B, C, or D
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `port` | str | Build HAT port ("A", "B", "C", "D") |

### Methods

#### `start(speed)`

Start motor at specified speed.

```python
motor.start(20)  # Speed 20
motor.start(-20)  # Reverse at speed 20
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `speed` | int | Speed (-100 to 100) |

#### `stop()`

Stop motor.

```python
motor.stop()
```

#### `run_for_degrees(degrees, speed=None)`

Rotate motor by specified degrees.

```python
motor.run_for_degrees(90)  # Rotate 90°
motor.run_for_degrees(-45, speed=10)  # Rotate -45° at speed 10
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `degrees` | float | required | Degrees to rotate |
| `speed` | int | None | Speed (None = default) |

#### `get_position()`

Get current encoder position.

```python
position = motor.get_position()
print(f"Position: {position}°")
```

**Returns:** `float` (position in degrees)

---

## Color Sensor

LEGO color sensor via Build HAT. Best used through `SensorInput` for async access.

### Direct Usage

```python
from buildhat import ColorSensor

color = ColorSensor("D")  # Port D
```

### Methods

#### `get_color()`

Get detected color.

```python
color_name = color.get_color()
print(f"Color: {color_name}")
```

**Returns:** `str` (color name: "black", "white", "red", "yellow", "green", "blue", etc.)

#### `get_ambient_light()`

Get ambient light level.

```python
light = color.get_ambient_light()
```

**Returns:** `int` (light level)

### Via SensorInput (Recommended)

```python
from systems.sensors import SensorInput

sensors = SensorInput(color_sensor_port="D")

# Get color name
color = await sensors.get_color()

# Check for black (returns 1 or 0)
is_black = await sensors.is_black()
if is_black:
    print("Black detected!")
```

---

## Pin Reference

See [Hardware Guide](../hardware.md) for complete pin mappings.

### Common GPIO Pins

| Sensor | Typical Pin |
|--------|-------------|
| Ultrasonic | 26 |
| Line Finder Left | 16 |
| Line Finder Right | 5 |
| Hall Sensor | 12 |
| Button | 22 |

### Build HAT Ports

| Port | Typical Use |
|------|-------------|
| A | Front drive motor |
| B | Turn/steering motor |
| C | Auxiliary motor |
| D | Payload motor / Color sensor |

---

## See Also

- [Hardware Guide](../hardware.md)
- [Navigation System API](navigation.md)
- [Mobility System API](mobility.md)
