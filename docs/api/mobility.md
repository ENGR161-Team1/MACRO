# Mobility System API

> Motor control with safety features and encoder tracking

```python
from systems.mobility_system import MotionController
```

---

## MotionController

Motor control system with ultrasonic safety ring and encoder velocity tracking.

### Constructor

```python
controller = MotionController(
    front_motor="A",
    turn_motor="B",
    ultrasonic_pin=26,
    slowdown_distance=30.0,
    stopping_distance=15.0,
    forward_speed=20,
    forward_speed_slow=10
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `front_motor` | str | required | Build HAT port for front motor ("A"-"D") |
| `turn_motor` | str | required | Build HAT port for turn motor ("A"-"D") |
| `ultrasonic_pin` | int | 26 | GPIO pin for ultrasonic sensor |
| `slowdown_distance` | float | 30.0 | Distance to start slowing (cm) |
| `stopping_distance` | float | 15.0 | Distance to stop (cm) |
| `forward_speed` | int | 20 | Normal forward speed |
| `forward_speed_slow` | int | 10 | Reduced speed in slowdown zone |

### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `front_motor` | Motor | Front drive motor instance |
| `turn_motor` | Motor | Steering motor instance |
| `ultrasonic` | UltrasonicSensor | Ultrasonic sensor instance |
| `forward_speed` | int | Current forward speed setting |
| `forward_speed_slow` | int | Slow speed setting |
| `slowdown_distance` | float | Slowdown threshold (cm) |
| `stopping_distance` | float | Stop threshold (cm) |
| `motor_position` | float | Current motor encoder position (°) |
| `motor_velocity` | float | Current motor velocity (°/s) |
| `_prev_position` | float | Previous encoder position |

---

## Motor Control Methods

### `start(speed=None)`

Start the front motor at specified or default speed.

```python
controller.start()  # Use default forward_speed
controller.start(speed=15)  # Custom speed
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `speed` | int | None | Speed to use (None = forward_speed) |

### `stop()`

Stop all motors immediately.

```python
controller.stop()
```

### `reverse(speed=None)`

Start front motor in reverse.

```python
controller.reverse()  # Use default speed
controller.reverse(speed=10)  # Custom speed
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `speed` | int | None | Speed to use (None = forward_speed) |

### `turn_left(degrees)`

Turn steering motor left by specified degrees.

```python
controller.turn_left(45)  # Turn 45° left
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `degrees` | float | Degrees to turn |

### `turn_right(degrees)`

Turn steering motor right by specified degrees.

```python
controller.turn_right(45)  # Turn 45° right
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `degrees` | float | Degrees to turn |

### `straighten(central_pos=0)`

Return steering to center position.

```python
controller.straighten()  # Return to 0
controller.straighten(central_pos=5)  # Return to offset center
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `central_pos` | float | 0 | Center position in degrees |

---

## Safety Ring Methods

### `async start_safety_ring()`

Start the ultrasonic safety monitoring loop.

```python
await controller.start_safety_ring()
```

**Behavior:**
- Continuously monitors ultrasonic distance
- `distance > slowdown_distance`: Normal speed
- `stopping_distance < distance ≤ slowdown_distance`: Slow speed
- `distance ≤ stopping_distance`: Stop

### `async run_with_safety()`

Convenience method to start motor and safety ring together.

```python
await controller.run_with_safety()
```

Equivalent to:
```python
controller.start()
await controller.start_safety_ring()
```

---

## Encoder Tracking Methods

### `async update_motor_state(dt)`

Update motor position and velocity from encoder.

```python
await controller.update_motor_state(dt=0.1)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `dt` | float | Time step in seconds |

**Updates:**
- `motor_position` - Current encoder position (°)
- `motor_velocity` - Calculated velocity (°/s)

### `async get_velocity()`

Get current motor velocity.

```python
velocity = await controller.get_velocity()
print(f"Motor velocity: {velocity:.2f} °/s")
```

**Returns:** `float` (velocity in degrees/second)

### `get_position()`

Get current motor encoder position.

```python
position = controller.get_position()
print(f"Motor position: {position:.2f}°")
```

**Returns:** `float` (position in degrees)

---

## Examples

### Basic Motor Control

```python
from systems.mobility_system import MotionController

controller = MotionController(
    front_motor="A",
    turn_motor="B"
)

# Start forward
controller.start()

# Turn left 45°
controller.turn_left(45)

# Stop
controller.stop()
```

### With Safety Ring

```python
import asyncio
from systems.mobility_system import MotionController

controller = MotionController(
    front_motor="A",
    turn_motor="B",
    ultrasonic_pin=26,
    slowdown_distance=30.0,
    stopping_distance=15.0
)

async def main():
    try:
        await controller.run_with_safety()
    except KeyboardInterrupt:
        controller.stop()

asyncio.run(main())
```

### Motor Encoder Tracking

```python
import asyncio
from systems.mobility_system import MotionController

controller = MotionController(front_motor="A", turn_motor="B")

async def track_motor():
    while True:
        await controller.update_motor_state(dt=0.1)
        print(f"Position: {controller.motor_position:.1f}° "
              f"Velocity: {controller.motor_velocity:.1f}°/s")
        await asyncio.sleep(0.1)

async def main():
    controller.start()
    await track_motor()

asyncio.run(main())
```

### Integration with Navigation

```python
import asyncio
from basehat import IMUSensor
from systems.navigation_system import Navigation3D
from systems.mobility_system import MotionController

async def main():
    imu = IMUSensor()
    motion = MotionController(front_motor="A", turn_motor="B")
    
    navigator = Navigation3D(
        imu=imu,
        motion_controller=motion,
        motor_velocity_threshold=1.0
    )
    
    await navigator.calibrate()
    motion.start()
    
    async def update_motors():
        while True:
            await motion.update_motor_state(dt=0.1)
            await asyncio.sleep(0.1)
    
    await asyncio.gather(
        navigator.run_continuous_update(),
        motion.start_safety_ring(),
        update_motors()
    )

asyncio.run(main())
```

---

## Manual Control Example

From `tests/mobility_test.py`:

```python
async def manual_controller():
    """Manual keyboard control."""
    central_pos = controller.turn_motor.get_position()
    
    while True:
        command = input("Command: ").strip().lower()
        
        if command == "w":
            controller.start()
        elif command == "s":
            controller.stop()
        elif command == "r":
            controller.reverse()
        elif command == "a":
            controller.turn_left(45)
        elif command == "d":
            controller.turn_right(45)
        elif command == "q":
            controller.straighten(central_pos)
        elif command == "x":
            break
```

**Available Commands:**
| Key | Action |
|-----|--------|
| `w` | Start forward |
| `s` | Stop all motors |
| `r` | Reverse |
| `a` | Turn left 45° |
| `d` | Turn right 45° |
| `q` | Straighten wheels |
| `+` | Increase speed |
| `-` | Decrease speed |
| `p` | Deploy payload |
| `o` | Retract payload |
| `l` | Stop payload |
| `x` | Exit |

---

## See Also

- [Navigation System API](navigation.md)
- [Display System API](display.md)
- [Hardware Guide](../hardware.md)
