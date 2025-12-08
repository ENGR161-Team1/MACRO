# Mobility System API

> Motor control with line following, safety features, and encoder tracking

```python
from systems.mobility_system import MotionController
```

---

## MotionController

Motor control system with line following, ultrasonic safety ring, and encoder tracking.

### Constructor

```python
controller = MotionController(
    front_motor="A",
    turn_motor="B",
    state=state,
    slowdown_distance=30.0,
    stopping_distance=15.0,
    forward_speed=20,
    forward_speed_slow=10,
    turn_speed=20,
    max_turn=100
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `front_motor` | str | `"A"` | Build HAT port for front motor |
| `turn_motor` | str | `"B"` | Build HAT port for turn motor |
| `state` | State | required | Centralized state object |
| `slowdown_distance` | float | `30.0` | Distance to start slowing (cm) |
| `stopping_distance` | float | `15.0` | Distance to stop (cm) |
| `forward_speed` | int | `20` | Normal forward speed |
| `forward_speed_slow` | int | `10` | Reduced speed in slowdown zone |
| `turn_speed` | int | `20` | Turn motor speed |
| `max_turn` | int | `100` | Maximum turn angle from center |

### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `front_motor` | Motor | Front drive motor instance |
| `turn_motor` | Motor | Steering motor instance |
| `state` | State | Shared state object |
| `central_pos` | float | Center position for steering |
| `moving` | bool | Whether robot is currently moving |
| `current_speed` | int | Current speed setting |
| `line_state` | str | Current line following state ("left", "center", "right") |
| `prev_left_in` | bool | Previous left line finder value |
| `prev_right_in` | bool | Previous right line finder value |

---

## Motor Control Methods

### `start(speed=None)`

Start the front motor at specified or default speed.

```python
controller.start()  # Use default forward_speed
controller.start(speed=15)  # Custom speed
```

### `stop()`

Stop all motors immediately.

```python
controller.stop()
```

### `async reverse()`

Start front motor in reverse.

```python
await controller.reverse()
```

---

## Turning Methods

### `async turn_left(amount=20)`

Turn left by a specified amount if within limits.

```python
await controller.turn_left(20)  # Turn 20° left
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `amount` | int | `20` | Degrees to turn |

### `async turn_right(amount=20)`

Turn right by a specified amount if within limits.

```python
await controller.turn_right(20)  # Turn 20° right
```

### `async turn_left_start()`

Start turning left continuously.

```python
await controller.turn_left_start()
```

### `async turn_right_start()`

Start turning right continuously.

```python
await controller.turn_right_start()
```

### `async stop_turn()`

Stop the turn motor.

```python
await controller.stop_turn()
```

### `async straighten()`

Straighten wheels back to central position.

```python
await controller.straighten()
```

### `async recalibrate_center()`

Set current turn position as the new center.

```python
await controller.recalibrate_center()
```

---

## Safety Methods

### `get_distance()`

Get distance from ultrasonic sensor via State.

```python
distance = controller.get_distance()
```

**Returns:** `float` - Distance in cm from `state.ultrasonic_distance`

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

Start motor and safety ring together.

```python
await controller.run_with_safety()
```

---

## Line Following

### `async auto_line_follow()`

Automatically follow a line using left and right line finders.

```python
await controller.auto_line_follow()
```

**Features:**
- Combines safety monitoring with line following
- Reads line finder values from State
- Pauses when `state.deploying_cargo` is True
- Uses state machine for smooth tracking

**Line State Machine:**

| State | Meaning | Action |
|-------|---------|--------|
| `"left"` | Robot is to the left of line | Turn right |
| `"center"` | Robot is centered on line | Straighten |
| `"right"` | Robot is to the right of line | Turn left |

**Transition Logic:**

When a sensor triggers from both clear:
- If previous state was opposite side → transition to `"center"`
- Otherwise → transition to opposite side

---

## Encoder Tracking

### `async update_motor_state()`

Update motor encoder state in State.

```python
await controller.update_motor_state()
```

**Updates:**
- `state.motor_position` - Front motor position (°)
- `state.motor_velocity` - Front motor velocity (°/s)
- `state.turn_position` - Turn motor position (°)

### `async run_update_loop(update_interval=0.1)`

Continuously update motor state at specified intervals.

```python
await controller.run_update_loop(update_interval=0.05)
```

---

## Usage Examples

### Basic Motor Control

```python
from systems.mobility_system import MotionController
from systems.state import State

state = State()
controller = MotionController(
    front_motor="A",
    turn_motor="B",
    state=state
)

# Start forward
controller.start()

# Turn left
await controller.turn_left(45)

# Stop
controller.stop()
```

### With Line Following

```python
import asyncio
from systems.state import State
from systems.sensors import SensorInput
from systems.mobility_system import MotionController

state = State()
sensors = SensorInput(state=state, line_finders=True)
controller = MotionController(state=state)

async def main():
    # Start sensor updates
    sensor_task = asyncio.create_task(sensors.run_sensor_update())
    
    # Run line following with safety
    await controller.auto_line_follow()

asyncio.run(main())
```

### With Controller

```python
from controller import Controller

async def main():
    controller = Controller()
    await controller.initialize()
    
    # auto_line_follow() starts automatically in run()
    await controller.run()
```

---

## State Integration

The MotionController reads from and writes to State:

**Reads:**
- `state.ultrasonic_distance` - For safety ring
- `state.lf_left_value` - Left line finder
- `state.lf_right_value` - Right line finder
- `state.deploying_cargo` - Pauses motion when True

**Writes:**
- `state.motor_position` - Front motor encoder position
- `state.motor_velocity` - Front motor velocity
- `state.turn_position` - Turn motor position

---

## See Also

- [State API](state.md) - State dataclass
- [Cargo API](cargo.md) - How `deploying_cargo` works
- [Controller API](controller.md) - How mobility is initialized
- [Architecture](../architecture.md) - System overview
