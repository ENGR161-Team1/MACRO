# Cargo System API

> Cargo detection and deployment using magnetic field sensing

```python
from systems.cargo_system import Cargo
```

---

## Cargo

Cargo detection and deployment system using magnetic field sensing.

### Constructor

```python
cargo = Cargo(
    state=state,
    motor_port="C",
    motor_speed=100,
    deploy_angle=180,
    edge_threshold=400.0,
    semi_threshold=1000.0,
    full_threshold=3000.0,
    required_detections=5,
    deploy_distance=0.24
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `state` | State | required | Centralized state object for sensor data |
| `motor_port` | str | `"B"` | Build HAT port for payload motor |
| `motor_speed` | int | `50` | Motor speed for deployment (0-100) |
| `deploy_angle` | int | `180` | Degrees to turn for deployment |
| `edge_threshold` | float | `400.0` | Magnetic threshold for edge detection (µT) |
| `semi_threshold` | float | `1000.0` | Magnetic threshold for semi-detection (µT) |
| `full_threshold` | float | `3000.0` | Magnetic threshold for full detection (µT) |
| `required_detections` | int | `5` | Consecutive full detections needed before deploying |
| `deploy_distance` | float | `0.0` | Distance to travel past max magnetic reading before deploying (m) |

### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `state` | State | Shared state object |
| `motor` | Motor | Build HAT motor instance |
| `deployed` | bool | Whether cargo has been deployed (prevents re-deployment) |
| `max_mag_delta` | float | Maximum magnetic delta observed during run |
| `max_mag_distance` | float | Distance traveled when max magnetic reading occurred |
| `deploy_distance` | float | Distance to travel past max reading before deploying (cm) |

---

## Methods

### `get_magnetic_delta()`

Return the magnetic field difference from baseline.

```python
delta = cargo.get_magnetic_delta()
```

**Returns:** `float` - Difference from baseline in micro-tesla (µT)

**Notes:**
- Returns raw magnetic field if not calibrated
- Uses `state.bias["mag"]` as baseline after calibration

---

### `detect_cargo_level()`

Detect the cargo proximity level based on magnetic field.

```python
level = cargo.detect_cargo_level()
```

**Returns:** `str` - One of `"none"`, `"edge"`, `"semi"`, or `"full"`

**Threshold Logic:**
```python
if mag_delta >= full_threshold:      # 3000 µT
    return "full"
elif mag_delta >= semi_threshold:    # 1000 µT
    return "semi"
elif mag_delta >= edge_threshold:    # 400 µT
    return "edge"
else:
    return "none"
```

---

### `is_cargo_detected()`

Check if any cargo is detected.

```python
if cargo.is_cargo_detected():
    print("Cargo nearby!")
```

**Returns:** `bool` - True if cargo detected at any level (edge, semi, or full)

---

### `async deploy()`

Deploy the cargo by turning the payload motor.

```python
await cargo.deploy()
```

**Behavior:**
1. If already deployed, prints message and returns
2. Sets `state.deploying_cargo = True` (pauses robot motion)
3. Turns motor by `+deploy_angle` degrees (blocking)
4. Sets `deployed = True`
5. Sets `state.deploying_cargo = False`

**Notes:**
- Uses `blocking=True` so robot stops during deployment
- Only deploys once - subsequent calls do nothing

---

### `async close()`

Close the payload bay by turning the motor back.

```python
await cargo.close()
```

**Behavior:**
1. Sets `state.deploying_cargo = True` (pauses robot motion)
2. Turns motor by `-deploy_angle` degrees (blocking)
3. Sets `state.deploying_cargo = False`

**Notes:**
- Does NOT reset `deployed` flag - prevents re-deployment
- Uses `blocking=True` so robot stops during closing

---

### `stop()`

Stop the cargo motor immediately.

```python
cargo.stop()
```

---

### `async run_cargo_update_loop(update_interval=0.1)`

Continuously monitor cargo level and auto-deploy when full.

```python
await cargo.run_cargo_update_loop(update_interval=0.1)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_interval` | float | `0.1` | Time between checks in seconds |

**Behavior:**
1. Updates `state.mag_delta` and `state.cargo_level` each iteration
2. When `cargo_level == "full"` and not yet deployed:
   - Increments `full_detection_count`
   - After `required_detections` consecutive full readings:
     - Calls `deploy()`
     - Waits 0.5 seconds
     - Calls `close()`
3. Resets counter if detection level drops

**Debouncing:**
- Prevents false positives from motor EMF interference
- Requires sustained detection before deploying
- Default: 5 consecutive readings at 100ms = 500ms sustained detection

---

## Usage Examples

### Basic Cargo Detection

```python
from systems.cargo_system import Cargo
from systems.state import State

state = State()
cargo = Cargo(state=state, motor_port="C")

# Check cargo level
level = cargo.detect_cargo_level()
print(f"Cargo level: {level}")
```

### Manual Deployment

```python
async def deploy_cargo():
    await cargo.deploy()
    await asyncio.sleep(0.5)
    await cargo.close()
```

### Continuous Monitoring with Controller

```python
from controller import Controller

async def main():
    controller = Controller()
    await controller.initialize()
    
    # Cargo monitoring starts automatically in controller.run()
    await controller.run()
```

---

## Configuration

In `macro_config.toml`:

```toml
[cargo.motor]
port = "C"          # Build HAT port
speed = 100         # Motor speed (0-100)
deploy_angle = 180  # Degrees to turn

[cargo.thresholds]
edge = 400.0        # µT for edge detection
semi = 1000.0       # µT for semi-detection
full = 3000.0       # µT for full detection

[cargo.detection]
required_consecutive = 5  # Debounce count
```

---

## State Integration

The Cargo system reads from and writes to State:

**Reads:**
- `state.magnetic_field` - Current magnetometer reading
- `state.calibrated_mag` - Whether magnetometer is calibrated
- `state.bias["mag"]` - Baseline magnetic field

**Writes:**
- `state.mag_delta` - Difference from baseline
- `state.cargo_level` - Current detection level
- `state.deploying_cargo` - True during deploy/close operations

---

## See Also

- [State API](state.md) - State dataclass documentation
- [Mobility API](mobility.md) - How `deploying_cargo` pauses motion
- [Architecture](../architecture.md) - System overview
