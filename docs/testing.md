# Testing Guide

> Running tests and testing components for MARCO

---

## Test Structure

```
tests/
├── mobility_test.py      # Motor and motion testing
├── nav_display_test.py   # Navigation display testing
└── ...
```

---

## mobility_test.py

Interactive motor testing with keyboard controls.

### Running the Test

```bash
cd /path/to/MARCO
python -m tests.mobility_test
```

### Mode Selection

On startup, select a mode:

```
Select mode (automatic/manual) [automatic]:
```

- **automatic**: Motors operate within safety ring, stop when approaching boundary
- **manual**: Direct motor control without safety constraints

### Keyboard Controls

| Key | Action |
|-----|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Turn left |
| `d` | Turn right |
| `q` | Stop all motors |
| `p` | Deploy payload |
| `o` | Retract payload |
| `l` | Stop payload |
| `+` / `=` | Zoom in display |
| `-` | Zoom out display |
| `Esc` | Exit |

### Example Session

```
$ python -m tests.mobility_test
Select mode (automatic/manual) [automatic]: manual
Starting mobility test in manual mode...

Controls:
  w/s: Forward/Backward
  a/d: Turn Left/Right
  q: Stop
  p/o/l: Payload Deploy/Retract/Stop
  +/-: Zoom In/Out
  Esc: Exit
```

---

## nav_display_test.py

Test the navigation display without physical hardware.

### Running the Test

```bash
python -m tests.nav_display_test
```

### Features Tested

- Real-time position display
- Heading indicator (arrow)
- Magnetic field ring
- Trail/path history
- Zoom controls
- Window resizing

### Mock Navigation

The test creates simulated movement patterns:

```python
# Circular path simulation
async def simulate_movement():
    t = 0
    while True:
        x = 5 * math.cos(t)
        y = 5 * math.sin(t)
        nav.location.x = x
        nav.location.y = y
        t += 0.02
        await asyncio.sleep(0.05)
```

---

## Writing New Tests

### Test File Template

```python
"""Test module description."""

import asyncio
from systems import Navigation3D, MotionController
from ui import NavigationDisplay


async def run_test():
    """Main test coroutine."""
    # Initialize systems
    nav = Navigation3D()
    motion = MotionController()
    
    # Create display
    display = NavigationDisplay(nav)
    
    try:
        # Test logic here
        while True:
            nav.update_state(0.02)
            motion.update(0.02)
            display.update()
            await asyncio.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        motion.stop_all()
        display.close()


if __name__ == "__main__":
    asyncio.run(run_test())
```

### Testing Individual Components

#### Test IMU Sensor

```python
from basehat import IMUSensor

imu = IMUSensor()

# Read values
accel = imu.getAccel()
gyro = imu.getGyro()
mag = imu.getMag()

print(f"Accel: {accel}")
print(f"Gyro: {gyro}")
print(f"Mag: {mag}")
```

#### Test Motor

```python
from buildhat import Motor

motor = Motor("A")

# Test movement
motor.run_for_degrees(90, speed=20)
print(f"Position: {motor.get_position()}")

motor.stop()
```

#### Test Ultrasonic

```python
from basehat import UltrasonicSensor

us = UltrasonicSensor(26)

for _ in range(10):
    print(f"Distance: {us.get_distance():.1f} cm")
    time.sleep(0.5)
```

---

## Testing Best Practices

### 1. Always Use Try/Finally

Ensure motors stop on exit:

```python
try:
    # Test code
    pass
finally:
    motion.stop_all()
```

### 2. Use Manual Mode for Initial Testing

Start with manual mode to understand motor behavior:

```python
if mode == "manual":
    # No safety constraints
    motion.move_forward(speed)
else:
    # Safety ring enforced
    if within_safety_ring:
        motion.move_forward(speed)
```

### 3. Test Components in Isolation

Test sensors and motors separately before integration:

```python
# First: Test sensor alone
imu = IMUSensor()
print(imu.getAccel())

# Then: Test motor alone
motor = Motor("A")
motor.run_for_degrees(45)

# Finally: Test together
nav = Navigation3D(motor_controller=motion)
```

### 4. Use Display for Debugging

The navigation display helps visualize system state:

```python
display = NavigationDisplay(nav, title="Debug View")

# Enable verbose output
nav.print_state(["position", "velocity", "heading"])
```

---

## Continuous Integration

### Running All Tests

```bash
# From project root
python -m pytest tests/
```

### Running Specific Test

```bash
python -m pytest tests/mobility_test.py -v
```

### Test with Coverage

```bash
python -m pytest tests/ --cov=systems --cov=ui
```

---

## Troubleshooting Tests

### "No module named 'buildhat'"

Hardware libraries require Raspberry Pi:

```bash
# On Raspberry Pi
pip install buildhat

# For mock testing (non-Pi)
# Create mock modules in tests/mocks/
```

### Display Not Updating

Ensure `update()` is called in the event loop:

```python
async def main_loop():
    while True:
        display.update()  # Must be called regularly
        await asyncio.sleep(0.02)
```

### Motor Not Responding

1. Check Build HAT power
2. Verify port connection
3. Test with simple script:

```python
from buildhat import Motor
m = Motor("A")
m.start(20)
time.sleep(1)
m.stop()
```

---

## See Also

- [Getting Started](getting-started.md)
- [Mobility API](api/mobility.md)
- [Navigation API](api/navigation.md)
