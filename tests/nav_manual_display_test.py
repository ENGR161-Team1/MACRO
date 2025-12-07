"""
nav_manual_display_test.py

Manual control test with integrated Navigation and NavigationDisplay visualization.

Uses keyboard input (w/a/s/d/r/q) for rover control while displaying
real-time navigation data.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncio
from fixtures import (
    create_sensors, SensorConfig,
    create_navigator, NavigationConfig,
    create_motion_controller, MobilityConfig
)
from ui.navigation_display import NavigationDisplay

# ============ CONFIGURATION ============
sensor_config = SensorConfig(
    imu=True,
    ultrasonic=True,
    ultrasonic_pin=26,
)

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

# Manual control settings
TURN_AMOUNT = 45  # degrees to turn per a/d press
MAX_TURN = 100    # max turn angle from center

# Enable/disable features
ENABLE_NAVIGATION = True
ENABLE_DISPLAY = True
# =======================================

# Create shared sensors instance
sensors = create_sensors(sensor_config)

# Create motion controller and navigator with shared sensors
motion = create_motion_controller(mobility_config, sensors=sensors)
navigator = create_navigator(nav_config, sensors=sensors, motion_controller=motion)
display = NavigationDisplay(
    width=display_config["width"],
    height=display_config["height"],
    scale=display_config["scale"],
    navigator=navigator
)

# Store central position for straightening
central_pos = 0


def print_help():
    """Print available commands."""
    print("\n=== Manual Control Commands ===")
    print("  w  - Start forward")
    print("  s  - Stop motors")
    print("  r  - Reverse")
    print("  a  - Turn left")
    print("  d  - Turn right")
    print("  q  - Straighten wheels")
    print("  +  - Increase speed")
    print("  -  - Decrease speed")
    print("  h  - Show this help")
    print("  x  - Exit")
    print("================================\n")


async def start_forward():
    """Start moving forward."""
    motion.front_motor.start(mobility_config.forward_speed)
    print(f"Started forward at speed {mobility_config.forward_speed}")


async def stop_motors():
    """Stop all motors."""
    motion.front_motor.stop()
    motion.turn_motor.stop()
    print("Stopped motors")


async def reverse():
    """Start moving in reverse."""
    motion.front_motor.stop()
    motion.front_motor.start(-mobility_config.forward_speed)
    print(f"Started reverse at speed {mobility_config.forward_speed}")


async def turn_left():
    """Turn left by TURN_AMOUNT degrees."""
    global central_pos
    turn_pos = motion.turn_motor.get_position()
    if turn_pos > central_pos - MAX_TURN:
        motion.turn_motor.run_for_degrees(-TURN_AMOUNT)
        print(f"Turned left. Position: {motion.turn_motor.get_position()}")
    else:
        print(f"Max left turn reached ({MAX_TURN}°)")


async def turn_right():
    """Turn right by TURN_AMOUNT degrees."""
    global central_pos
    turn_pos = motion.turn_motor.get_position()
    if turn_pos < central_pos + MAX_TURN:
        motion.turn_motor.run_for_degrees(TURN_AMOUNT)
        print(f"Turned right. Position: {motion.turn_motor.get_position()}")
    else:
        print(f"Max right turn reached ({MAX_TURN}°)")


async def straighten():
    """Return wheels to center position."""
    global central_pos
    turn_pos = motion.turn_motor.get_position()
    offset = central_pos - turn_pos
    if abs(offset) > 1:
        motion.turn_motor.run_for_degrees(offset)
        print(f"Straightened. Offset was: {offset}°")
    else:
        print("Already straight")


async def increase_speed():
    """Increase forward speed."""
    mobility_config.forward_speed += 5
    print(f"Speed increased to {mobility_config.forward_speed}")


async def decrease_speed():
    """Decrease forward speed."""
    if mobility_config.forward_speed > 5:
        mobility_config.forward_speed -= 5
        print(f"Speed decreased to {mobility_config.forward_speed}")
    else:
        print("Already at minimum speed")


async def manual_controller():
    """Handle manual keyboard input for rover control."""
    global central_pos
    central_pos = motion.turn_motor.get_position()
    print_help()
    
    loop = asyncio.get_event_loop()
    
    while True:
        # Run input() in executor to not block async loop
        command = await loop.run_in_executor(None, lambda: input("Command: ").strip().lower())
        
        if command == "w":
            await start_forward()
        elif command == "s":
            await stop_motors()
        elif command == "r":
            await reverse()
        elif command == "a":
            await turn_left()
        elif command == "d":
            await turn_right()
        elif command == "q":
            await straighten()
        elif command == "+" or command == "=":
            await increase_speed()
        elif command == "-":
            await decrease_speed()
        elif command == "h":
            print_help()
        elif command == "x":
            print("Exiting...")
            break
        else:
            print(f"Unknown command: '{command}'. Press 'h' for help.")


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
    """Calibrate IMU, then run manual control, navigation, and display concurrently."""
    # Calibrate IMU while stationary (before motors start)
    print("Calibrating IMU... keep rover stationary.")
    await navigator.calibrate(samples=nav_config.calibration_samples)
    print("Calibration complete!\n")
    
    # Build task list
    tasks = [
        start_motor_tracking(),
        manual_controller(),
    ]
    
    if ENABLE_NAVIGATION:
        tasks.append(start_navigation())
    
    if ENABLE_DISPLAY:
        tasks.append(start_display())
    
    # Run concurrently - will exit when manual_controller returns
    await asyncio.gather(*tasks, return_exceptions=True)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        motion.stop()
        display.close()
        print(f"\nProgram terminated. Motors stopped.")
        if navigator.log:
            print(f"Logged {len(navigator.log)} navigation entries over "
                  f"{navigator.log[-1]['timestamp']:.2f} seconds.")
