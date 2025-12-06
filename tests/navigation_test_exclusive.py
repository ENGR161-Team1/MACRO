"""
navigation_test_exclusive.py

Navigation-only test without mobility.

Tests Navigation3D position tracking using the IMU sensor
without any motor control or safety ring.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncio
from basehat import IMUSensor, HallSensor
from systems.navigation_system import Navigation3D
import math
import time

# Navigation
imu_sensor = IMUSensor()
# hall_sensor = HallSensor(22)
navigator = Navigation3D(imu=imu_sensor, mode="degrees")


async def main():
    """Calibrate IMU and run navigation updates."""
    await navigator.run_continuous_update(
        update_interval=0.1,
        log_state=False,
        print_state=True,
        calibrate=True
    )

def check_magnetism():
    while True:
        x_mag, y_mag, z_mag = imu_sensor.getMag()
        imu_mag = math.sqrt(x_mag ** 2 + y_mag ** 2 + z_mag ** 2)
        # hall_mag = hall_sensor.value
        print(imu_mag)
        time.sleep(0.2)


if __name__ == "__main__":
    try:
        # asyncio.run(main())
        check_magnetism()
    except KeyboardInterrupt:
        print(f"\nProgram terminated.")
        if navigator.log:
            print(f"Logged {len(navigator.log)} navigation entries over "
                  f"{navigator.log[-1]['timestamp']:.2f} seconds.")
