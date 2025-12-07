# Sensor APIs

> Hardware sensor interfaces for Grove Base HAT

```python
from basehat import IMUSensor, UltrasonicSensor, LineFinder, HallSensor, Button
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

## HallSensor

Hall effect magnetic position sensor.

### Constructor

```python
from basehat import HallSensor

hall = HallSensor(pin=12)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pin` | int | GPIO pin number |

### Methods

#### `read()`

Read magnetic field presence.

```python
value = hall.read()
```

**Returns:** `bool` or `int` (sensor reading)

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

LEGO color sensor via Build HAT.

### Constructor

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

**Returns:** `str` (color name)

#### `get_ambient_light()`

Get ambient light level.

```python
light = color.get_ambient_light()
```

**Returns:** `int` (light level)

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
