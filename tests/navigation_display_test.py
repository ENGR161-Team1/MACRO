"""
navigation_display_test.py

Mobility test with integrated Navigation3D and NavigationDisplay visualization.

This combines the safety ring obstacle avoidance from MotionController
with real-time navigation tracking and visual display.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncio
from fixtures import (
    create_navigator, NavigationConfig,
    create_motion_controller, MobilityConfig
)
from ui.navigation_display import NavigationDisplay

# ============ CONFIGURATION ============
nav_config = NavigationConfig(
    update_interval=0.1,
    log_state=True,
    print_state=False,  # Display handles visualization
    print_fields=["all"],
    calibrate=False,  # Calibrated manually before motors start
    calibration_samples=50,
    velocity_decay=0.04,
    accel_threshold=0.05,
    motor_velocity_threshold=1.0,
)

mobility_config = MobilityConfig(
    front_motor="A",
    turn_motor="B",
    ultrasonic_pin=26,
    slowdown_distance=30.0,
    stopping_distance=15.0,
    forward_speed=20,
    forward_speed_slow=10,
)

display_config = {
    "width": 800,
    "height": 800,
    "scale": 50.0,  # pixels per meter
}

# Enable/disable features
ENABLE_SAFETY_RING = True
ENABLE_NAVIGATION = True
ENABLE_DISPLAY = True
ENABLE_MOBILITY = True
# =======================================

motion = create_motion_controller(mobility_config)
navigator = create_navigator(nav_config, motion_controller=motion)
display = NavigationDisplay(
    width=display_config["width"],
    height=display_config["height"],
    scale=display_config["scale"],
    navigator=navigator
)


async def start_navigation():
    """Run continuous navigation updates with logging."""
    await navigator.run_continuous_update(
        update_interval=nav_config.update_interval,
        log_state=nav_config.log_state,
        print_state=nav_config.print_state,
        calibrate=False
    )


async def start_display():
    """Run the navigation display with live updates from navigator."""
    await display.run_continuous(update_interval=nav_config.update_interval)


async def start_motor_tracking():
    """Update motor encoder state continuously."""
    while True:
        await motion.update_motor_state(dt=nav_config.update_interval)
        await asyncio.sleep(nav_config.update_interval)


async def main():
    """Calibrate IMU, then run safety ring, navigation, and display concurrently."""
    # Calibrate IMU while stationary (before motors start)
    await navigator.calibrate(samples=nav_config.calibration_samples)
    
    # Start the motor if mobility enabled
    if ENABLE_MOBILITY:
        motion.start()
    
    # Build task list
    tasks = [start_motor_tracking()]
    
    if ENABLE_SAFETY_RING and ENABLE_MOBILITY:
        tasks.append(motion.start_safety_ring())
    
    if ENABLE_NAVIGATION:
        tasks.append(start_navigation())
    
    if ENABLE_DISPLAY:
        tasks.append(start_display())
    
    # Run concurrently
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        motion.stop()
        display.close()
        print(f"\nProgram terminated. Motors stopped.")
        if navigator.log:
            print(f"Logged {len(navigator.log)} navigation entries over "
                  f"{navigator.log[-1]['timestamp']:.2f} seconds.")
