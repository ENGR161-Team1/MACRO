# Display System API

> Real-time navigation visualization with zoom and magnetic field display

```python
from ui.navigation_display import NavigationDisplay
```

---

## NavigationDisplay

Real-time 2D visualization of rover position, velocity, and magnetic field.

### Constructor

```python
display = NavigationDisplay(
    width=800,
    height=800,
    world_min=-10.0,
    world_max=10.0,
    title="MACRO Navigation Display",
    navigator=navigator
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `width` | int | 800 | Initial window width (pixels) |
| `height` | int | 800 | Initial window height (pixels) |
| `world_min` | float | -10.0 | Minimum world coordinate (meters) |
| `world_max` | float | 10.0 | Maximum world coordinate (meters) |
| `title` | str | "MACRO Navigation Display" | Window title |
| `navigator` | Navigation3D | None | Navigation instance for auto-updates |

### Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `width` | int | Current window width |
| `height` | int | Current window height |
| `scale` | float | Pixels per meter (auto-calculated) |
| `world_min` | float | Minimum world bound |
| `world_max` | float | Maximum world bound |
| `position` | tuple | Current position (x, y, z) |
| `orientation` | tuple | Current orientation (yaw, pitch, roll) |
| `velocity` | tuple | Current velocity (vx, vy, vz) |
| `acceleration` | tuple | Current acceleration (ax, ay, az) |
| `magnetic_magnitude` | float | Magnetic field magnitude (µT) |
| `motor_velocity` | float | Motor velocity (°/s) |
| `navigator` | Navigation3D | Linked navigation instance |
| `running` | bool | Whether display is active |

---

## Display Methods

### `show()`

Create and display the window.

```python
display.show()
```

Creates:
- Info panel with position, orientation, velocity, acceleration, magnetic field, motor velocity
- Canvas with grid and rover visualization

### `close()`

Close the display window.

```python
display.close()
```

### `process_events()`

Process pending GUI events (for manual main loops).

```python
display.process_events()
```

---

## Update Methods

### `update(**kwargs)`

Update display with new data manually.

```python
display.update(
    position=(1.0, 2.0, 0.0),
    orientation=(45.0, 0.0, 0.0),
    velocity=(0.5, 0.3, 0.0),
    acceleration=(0.1, 0.0, 0.0),
    magnetic_magnitude=120.5,
    motor_velocity=15.0
)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `position` | tuple | (x, y, z) in meters |
| `orientation` | tuple | (yaw, pitch, roll) in degrees |
| `velocity` | tuple | (vx, vy, vz) in m/s |
| `acceleration` | tuple | (ax, ay, az) in m/s² |
| `magnetic_magnitude` | float | Magnetic field in µT |
| `motor_velocity` | float | Motor velocity in °/s |

### `update_from_navigator()`

Update display from linked Navigation3D instance.

```python
display.update_from_navigator()
```

Automatically reads:
- `navigator.pos`
- `navigator.orientation`
- `navigator.velocity`
- `navigator.acceleration`
- `navigator.magnetic_magnitude`
- `navigator.motion_controller.motor_velocity` (if available)

### `async run_continuous(update_interval=0.1)`

Run continuous update loop.

```python
await display.run_continuous(update_interval=0.1)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_interval` | float | 0.1 | Time between updates (seconds) |

---

## Zoom Methods

### `zoom_in(factor=1.5)`

Zoom in by reducing world bounds.

```python
display.zoom_in()  # 1.5x zoom
display.zoom_in(factor=2.0)  # 2x zoom
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `factor` | float | 1.5 | Zoom factor |

### `zoom_out(factor=1.5)`

Zoom out by expanding world bounds.

```python
display.zoom_out()  # 1.5x zoom out
display.zoom_out(factor=2.0)  # 2x zoom out
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `factor` | float | 1.5 | Zoom factor |

### `set_world_bounds(world_min, world_max)`

Set world bounds programmatically.

```python
display.set_world_bounds(-5.0, 5.0)  # -5m to 5m view
display.set_world_bounds(-20.0, 20.0)  # -20m to 20m view
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `world_min` | float | Minimum coordinate (meters) |
| `world_max` | float | Maximum coordinate (meters) |

---

## Keyboard Controls

When the display window is focused:

| Key | Action |
|-----|--------|
| `+` or `=` | Zoom in |
| `-` | Zoom out |
| Mouse wheel | Zoom in/out |

---

## Visual Elements

### Grid

| Element | Color | Description |
|---------|-------|-------------|
| Minor grid | `#101010` | 0.1m spacing |
| Major grid | `#404040` | 1.0m spacing |
| Axes | `#606060` | X and Y axes |

### Rover

| Element | Color | Description |
|---------|-------|-------------|
| Position dot | Black | Current position |
| Velocity arrow | Blue (`#0066FF`) | Velocity direction and magnitude |
| Magnetic ring | White→Blue | Field intensity (0-5000 µT) |

### Info Panel

Shows:
- Position (x, y, z) in meters
- Orientation (yaw, pitch, roll) in degrees
- Velocity (vx, vy, vz) in m/s
- Acceleration (ax, ay, az) in m/s²
- Magnetic Field in µT
- Motor Velocity in °/s

---

## Examples

### Basic Display

```python
from ui.navigation_display import NavigationDisplay

display = NavigationDisplay()
display.show()

# Manual update
display.update(position=(1.0, 2.0, 0.0))
display.process_events()
```

### With Navigator

```python
import asyncio
from basehat import IMUSensor
from systems.navigation_system import Navigation3D
from ui.navigation_display import NavigationDisplay

imu = IMUSensor()
navigator = Navigation3D(imu=imu)
display = NavigationDisplay(navigator=navigator)

async def main():
    await navigator.calibrate()
    
    await asyncio.gather(
        navigator.run_continuous_update(update_interval=0.1),
        display.run_continuous(update_interval=0.1)
    )

asyncio.run(main())
```

### Custom World Bounds

```python
# Smaller view (-5m to 5m)
display = NavigationDisplay(
    world_min=-5.0,
    world_max=5.0,
    navigator=navigator
)

# Larger view (-50m to 50m)
display = NavigationDisplay(
    world_min=-50.0,
    world_max=50.0,
    navigator=navigator
)
```

### Dynamic Zoom

```python
# Programmatic zoom
display.zoom_in(2.0)  # 2x zoom in
display.zoom_out(1.5)  # 1.5x zoom out

# Set exact bounds
display.set_world_bounds(-2.0, 2.0)  # Focus on small area
```

---

## Magnetic Field Visualization

The magnetic field is displayed as a ring around the rover:

| Magnitude | Ring Color |
|-----------|------------|
| 0 µT | White (#FFFFFF) |
| 2500 µT | Light blue |
| 5000 µT | Pure blue (#0000FF) |

The ring radius is 0.06 meters in world coordinates (scales with zoom).

---

## See Also

- [Navigation System API](navigation.md)
- [Mobility System API](mobility.md)
- [Testing Guide](../testing.md)
