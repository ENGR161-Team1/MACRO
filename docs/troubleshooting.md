# Troubleshooting Guide

> Common issues and solutions for MARCO

---

## Installation Issues

### "No module named 'buildhat'"

**Cause:** Build HAT library not installed or not on Raspberry Pi.

**Solution:**
```bash
pip install buildhat
```

Note: The `buildhat` package only works on Raspberry Pi with the Build HAT attached.

### "No module named 'smbus2'"

**Cause:** I2C library missing for Grove sensors.

**Solution:**
```bash
pip install smbus2
```

### "Permission denied" for GPIO

**Cause:** User not in gpio group.

**Solution:**
```bash
sudo usermod -a -G gpio $USER
# Log out and back in
```

---

## I2C / IMU Issues

### IMU Not Detected

**Symptoms:**
- `i2cdetect` shows no device at 0x68
- `IMUSensor()` raises connection error

**Solutions:**

1. **Enable I2C:**
   ```bash
   sudo raspi-config
   # Interface Options → I2C → Enable
   sudo reboot
   ```

2. **Check wiring:**
   - SDA to SDA (GPIO2)
   - SCL to SCL (GPIO3)
   - VCC to 3.3V
   - GND to GND

3. **Verify detection:**
   ```bash
   i2cdetect -y 1
   ```

### IMU Returns Constant Values

**Cause:** Sensor needs initialization time.

**Solution:** Add delay after initialization:
```python
imu = IMUSensor()
time.sleep(0.5)  # Wait for sensor to stabilize
accel = imu.getAccel()
```

### Magnetometer Reading Spikes

**Cause:** Nearby magnetic interference or uncalibrated sensor.

**Solution:** Calibrate magnetometer:
```python
await navigator.calibrate(samples=100)
```

Move sensor away from motors, speakers, or metal objects during calibration.

---

## Motor Issues

### Motor Not Responding

**Possible Causes:**

1. **No power to Build HAT:**
   - Check 8V power connection
   - Verify LED on Build HAT is lit

2. **Wrong port:**
   ```python
   # Check port letter
   motor = Motor("A")  # A, B, C, or D
   ```

3. **Cable disconnected:**
   - Check LPF2 cable at both ends
   - Try a different cable

**Test Script:**
```python
from buildhat import Motor
motor = Motor("A")
motor.run_for_degrees(90)  # Should rotate 90°
```

### Motor Runs Continuously

**Cause:** `start()` called without `stop()`.

**Solution:**
```python
motor.start(20)
time.sleep(1)
motor.stop()  # Don't forget to stop!
```

### Encoder Position Drifting

**Cause:** Normal mechanical slippage or missed encoder counts.

**Solution:** Periodically reset position reference:
```python
current_pos = motor.get_position()
# Use relative movements
motor.run_for_degrees(target - current_pos)
```

---

## Navigation Issues

### Position Drifts Over Time

**Cause:** IMU integration error accumulates.

**Solutions:**

1. **Increase calibration samples:**
   ```python
   await navigator.calibrate(samples=200)
   ```

2. **Adjust velocity decay:**
   ```python
   navigator = Navigation3D(
       velocity_decay=0.06,  # Higher = faster decay
       motor_velocity_threshold=0.5
   )
   ```

3. **Use motor velocity tracking:**
   ```python
   navigator = Navigation3D(
       motion_controller=motion,
       motor_velocity_threshold=1.0
   )
   ```

### Heading Incorrect

**Cause:** Gyroscope drift or magnetic interference.

**Solutions:**

1. **Recalibrate with sensor stationary:**
   ```python
   print("Keep sensor still for calibration...")
   await navigator.calibrate(samples=100)
   ```

2. **Check for nearby magnets/motors:**
   - Mount IMU away from motors
   - Avoid metal surfaces

### Jerky Position Updates

**Cause:** Update interval too long or velocity spikes.

**Solution:**
```python
# Faster updates for smoother position
await navigator.run_continuous_update(
    update_interval=0.05  # 50ms instead of 100ms
)
```

---

## Display Issues

### Display Window Not Appearing

**Cause:** Tkinter not installed or no display.

**Solution:**
```bash
# Install tkinter
sudo apt-get install python3-tk

# For headless (SSH), enable X forwarding
ssh -X user@raspberrypi
```

### Display Freezes

**Cause:** `update()` not being called in async loop.

**Solution:**
```python
async def main_loop():
    while True:
        display.update()  # Must call regularly
        await asyncio.sleep(0.02)
```

### Rover Appears Off-Screen

**Cause:** World bounds don't contain rover position.

**Solution:**
```python
# Expand world bounds
display.set_world_bounds(-20, 20)  # Larger range

# Or zoom out
display.zoom_out()
display.zoom_out()
```

### Magnetic Ring Not Visible

**Cause:** Ring radius too small at current zoom level.

**Solution:**
```python
# Zoom in to see the 0.06m magnetic ring
display.zoom_in()
# Or set tighter world bounds
display.set_world_bounds(-1, 1)
```

---

## Ultrasonic Sensor Issues

### Reading Returns 0 or -1

**Cause:** No echo received (object too close or too far).

**Solutions:**

1. **Check distance range:** 2cm - 400cm typical
2. **Verify wiring:** Signal, VCC, GND
3. **Clear obstruction:** Ensure clear path

**Test:**
```python
from basehat import UltrasonicSensor
us = UltrasonicSensor(26)
for _ in range(10):
    print(f"Distance: {us.get_distance()}")
    time.sleep(0.2)
```

### Inconsistent Readings

**Cause:** Surface texture, angle, or interference.

**Solutions:**
- Point sensor perpendicular to surface
- Average multiple readings:
```python
readings = [us.get_distance() for _ in range(5)]
avg_distance = sum(readings) / len(readings)
```

---

## Async Issues

### "coroutine was never awaited"

**Cause:** Async function called without `await`.

**Wrong:**
```python
navigator.calibrate(samples=50)  # Missing await!
```

**Correct:**
```python
await navigator.calibrate(samples=50)
```

### "Event loop is already running"

**Cause:** Calling `asyncio.run()` inside existing loop.

**Solution:** Use `await` or create task:
```python
# If already in async context
await navigator.run_continuous_update()

# Or create task
task = asyncio.create_task(navigator.run_continuous_update())
```

### Program Hangs on Exit

**Cause:** Background tasks not cancelled.

**Solution:**
```python
try:
    await main_loop()
except KeyboardInterrupt:
    pass
finally:
    motion.stop_all()
    # Cancel all tasks
    for task in asyncio.all_tasks():
        task.cancel()
```

---

## Performance Issues

### Slow Update Rate

**Cause:** Too much processing per loop iteration.

**Solutions:**

1. **Reduce print frequency:**
   ```python
   # Print every 10th update instead of every update
   if update_count % 10 == 0:
       navigator.print_state()
   ```

2. **Simplify display:**
   ```python
   # Reduce trail length
   display.max_trail_length = 50  # Instead of default 100
   ```

### High CPU Usage

**Cause:** Busy loop without sleep.

**Solution:** Always include sleep in loops:
```python
while True:
    # ... processing ...
    await asyncio.sleep(0.02)  # Yield to other tasks
```

---

## Getting Help

If you can't resolve an issue:

1. **Check logs:** Enable verbose output
   ```python
   navigator.print_state(["position", "velocity", "heading", "magnetic"])
   ```

2. **Isolate the problem:** Test components individually

3. **Check hardware:** Verify all connections

4. **Review version:** Ensure you're on v0.8.2
   ```bash
   cat pyproject.toml | grep version
   ```

---

## See Also

- [Hardware Guide](hardware.md)
- [Testing Guide](testing.md)
- [Getting Started](getting-started.md)
