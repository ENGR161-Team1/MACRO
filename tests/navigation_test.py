"""
navigation_test.py

Navigation-only test without mobility.

Tests Navigation3D position tracking using the IMU sensor
without any motor control or safety ring.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncio
from fixtures import create_sensors, SensorConfig, create_navigator, NavigationConfig

# ============ CONFIGURATION ============
sensor_config = SensorConfig(
    imu=True,
    ultrasonic=False,  # Not needed for navigation-only
)

config = NavigationConfig(
    update_interval=0.1,
    log_state=False,
    print_state=True,
    print_fields=["position", "velocity"],
    calibrate=True,
    calibration_samples=50,
    velocity_decay=0.04,
    accel_threshold=0.05,
)

# Test mode: "navigation" or "magnetism"
TEST_MODE = "magnetism"
# =======================================

sensors = create_sensors(sensor_config)
navigator = create_navigator(config, sensors=sensors)


async def run_navigation():
    """Calibrate IMU and run navigation updates."""
    await navigator.run_continuous_update(
        update_interval=config.update_interval,
        log_state=config.log_state,
        print_state=config.print_state,
        calibrate=config.calibrate,
        calibration_samples=config.calibration_samples
    )


async def run_magnetism():
    """Check magnetic field magnitude using Navigation3D."""
    await navigator.calibrate(samples=config.calibration_samples)
    while True:
        mag = await navigator.get_magnetic_field()
        print(f"{mag:.2f} µT")
        await asyncio.sleep(0.2)


async def main():
    if TEST_MODE == "navigation":
        await run_navigation()
    elif TEST_MODE == "magnetism":
        await run_magnetism()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print(f"\nProgram terminated.")
        if navigator.log:
            print(f"Logged {len(navigator.log)} navigation entries over "
                  f"{navigator.log[-1]['timestamp']:.2f} seconds.")
