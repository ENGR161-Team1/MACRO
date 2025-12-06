import asyncio
from buildhat import ColorSensor, Motor
from systems.mobility_system import MotionController
# TODO: LineFinder class not yet implemented in basehat/line_finder.py
# from basehat import LineFinder

# Initialize motion controller with same settings as before
controller = MotionController(
    front_motor="A",
    turn_motor="B",
    ultrasonic_pin=26,
    slowdown_distance=30.0,
    stopping_distance=15.0,
    forward_speed=20,
    forward_speed_slow=10
)

# Payload motor
payload_motor = Motor("C")
PAYLOAD_SPEED = 2

color_sensor = ColorSensor("D")
# TODO: Uncomment when LineFinder is implemented
# lf_left = LineFinder(16)
# lf_right = LineFinder(5)
lf_left = None
lf_right = None

# Manual control settings
TURN_AMOUNT = 45  # degrees to turn per a/d press
MAX_TURN = 100    # max turn angle from center
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
    print("  p  - Deploy payload")
    print("  o  - Retract payload")
    print("  l  - Stop payload")
    print("  h  - Show this help")
    print("  x  - Exit")
    print("================================\n")


async def start_forward():
    """Start moving forward."""
    controller.front_motor.start(controller.forward_speed)
    print(f"Started forward at speed {controller.forward_speed}")


async def stop_motors():
    """Stop all motors."""
    controller.front_motor.stop()
    controller.turn_motor.stop()
    print("Stopped motors")


async def reverse():
    """Start moving in reverse."""
    controller.front_motor.stop()
    controller.front_motor.start(-controller.forward_speed)
    print(f"Started reverse at speed {controller.forward_speed}")


async def turn_left():
    """Turn left by TURN_AMOUNT degrees."""
    global central_pos
    turn_pos = controller.turn_motor.get_position()
    if turn_pos > central_pos - MAX_TURN:
        controller.turn_motor.run_for_degrees(-TURN_AMOUNT)
        print(f"Turned left. Position: {controller.turn_motor.get_position()}")
    else:
        print(f"Max left turn reached ({MAX_TURN}°)")


async def turn_right():
    """Turn right by TURN_AMOUNT degrees."""
    global central_pos
    turn_pos = controller.turn_motor.get_position()
    if turn_pos < central_pos + MAX_TURN:
        controller.turn_motor.run_for_degrees(TURN_AMOUNT)
        print(f"Turned right. Position: {controller.turn_motor.get_position()}")
    else:
        print(f"Max right turn reached ({MAX_TURN}°)")


async def straighten():
    """Return wheels to center position."""
    global central_pos
    turn_pos = controller.turn_motor.get_position()
    offset = central_pos - turn_pos
    if abs(offset) > 1:
        controller.turn_motor.run_for_degrees(offset)
        print(f"Straightened. Offset was: {offset}°")
    else:
        print("Already straight")


async def increase_speed():
    """Increase forward speed."""
    controller.forward_speed += 5
    print(f"Speed increased to {controller.forward_speed}")


async def decrease_speed():
    """Decrease forward speed."""
    if controller.forward_speed > 5:
        controller.forward_speed -= 5
        print(f"Speed decreased to {controller.forward_speed}")
    else:
        print("Already at minimum speed")


async def deploy_payload():
    """Deploy payload (start payload motor forward)."""
    payload_motor.start(PAYLOAD_SPEED)
    print("Deploying payload...")


async def retract_payload():
    """Retract payload (start payload motor reverse)."""
    payload_motor.start(-PAYLOAD_SPEED)
    print("Retracting payload...")


async def stop_payload():
    """Stop payload motor."""
    payload_motor.stop()
    print("Payload motor stopped")


async def manual_controller():
    """Handle manual keyboard input for rover control."""
    global central_pos
    central_pos = controller.turn_motor.get_position()
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
        elif command == "p":
            await deploy_payload()
        elif command == "o":
            await retract_payload()
        elif command == "l":
            await stop_payload()
        elif command == "h":
            print_help()
        elif command == "x":
            print("Exiting...")
            break
        else:
            print(f"Unknown command: '{command}'. Press 'h' for help.")


async def run_automatic():
    """Run automatic mode with safety ring."""
    await controller.run_with_safety()


async def run_manual():
    """Run manual control mode."""
    await manual_controller()


if __name__ == "__main__":
    print("=== Mobility Test ===")
    print("Select mode:")
    print("  1. Automatic (safety ring)")
    print("  2. Manual control")
    mode = input("Mode (1/2): ").strip()
    
    try:
        if mode == "2":
            asyncio.run(run_manual())
        else:
            asyncio.run(run_automatic())
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        payload_motor.stop()
        print("\nProgram terminated. Motors stopped.")