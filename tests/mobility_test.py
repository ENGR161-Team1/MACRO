import asyncio
from buildhat import ColorSensor
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

color_sensor = ColorSensor("D")
# TODO: Uncomment when LineFinder is implemented
# lf_left = LineFinder(16)
# lf_right = LineFinder(5)
lf_left = None
lf_right = None

if __name__ == "__main__":
    try:
        asyncio.run(controller.run_with_safety())
    except KeyboardInterrupt:
        controller.stop()
        print("Program terminated. Motors stopped.")