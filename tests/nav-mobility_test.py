"""
navigation_test.py

Mobility test with integrated Navigation3D position tracking.

This combines the safety ring obstacle avoidance from MotionController
with real-time navigation tracking using the IMU sensor.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncio
from basehat import IMUSensor
from systems.mobility_system import MotionController
from systems.navigation_system import Navigation3D# Motion controller (handles motors and safety ring)
motion = MotionController(
    front_motor="A",
    turn_motor="B",
    ultrasonic_pin=26,
    slowdown_distance=30.0,
    stopping_distance=15.0,
    forward_speed=20,
    forward_speed_slow=10
)

# Navigation
imu_sensor = IMUSensor()
navigator = Navigation3D(imu=imu_sensor, mode="degrees")


async def start_navigation():
    """Run continuous navigation updates with logging and printing."""
    await navigator.run_continuous_update(
        update_interval=0.1,
        log_state=True,
        print_state=True,
        calibrate=False  # Already calibrated in main
    )


async def main():
    """Calibrate IMU, then run safety ring and navigation concurrently."""
    # Calibrate IMU while stationary (before motors start)
    await navigator.calibrate(samples=50)
    
    # Now start the motor
    motion.start()
    
    # Run safety ring and navigation concurrently
    await asyncio.gather(
        motion.start_safety_ring(),
        start_navigation()
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        motion.stop()
        print(f"\nProgram terminated. Motors stopped.")
        if navigator.log:
            print(f"Logged {len(navigator.log)} navigation entries over "
                  f"{navigator.log[-1]['timestamp']:.2f} seconds.")
