import asyncio
from buildhat import Motor, ColorSensor
from basehat import UltrasonicSensor, LineFinder

front_motor = Motor("A")
turn_motor = Motor("B")
color_sensor = ColorSensor("D")
ultrasonic_sensor = UltrasonicSensor(26)
lf_left = LineFinder(16)
lf_right = LineFinder(5)
slowdown_distance = 30.0  # cm
stopping_distance = 15.0  # cm
forward_speed = 20
forward_speed_slow = 10
moving = True
current_speed = forward_speed

async def start_safety_ring():
    global moving, current_speed, slowdown_distance, stopping_distance
    while True:
        try:
            dist = float(ultrasonic_sensor.getDist)
        except:
            dist = 30.0
        # print(dist)
        if dist < stopping_distance:
            if moving:
                front_motor.stop()
                print("Obstacle detected! Stopping motors.")
                moving = False
        elif dist < slowdown_distance:
            if moving and current_speed != forward_speed_slow:
                front_motor.start(forward_speed_slow)
                print("Obstacle nearby! Slowing down.")
                current_speed = forward_speed_slow
            elif not moving:
                front_motor.start(forward_speed_slow)
                print("Path partially clear. Resuming movement at slow speed.")
                moving = True
                current_speed = forward_speed_slow
        else:
            if not moving:
                front_motor.start(forward_speed)
                print("Path clear. Resuming movement.")
                moving = True
                current_speed = forward_speed
            elif current_speed != forward_speed:
                front_motor.start(forward_speed)
                print("Path clear. Speeding up.")
                current_speed = forward_speed
        await asyncio.sleep(0.1)

if __name__ == "__main__":
    try:
        front_motor.start(forward_speed)
        asyncio.run(start_safety_ring())
    except KeyboardInterrupt:
        front_motor.stop()
        turn_motor.stop()
        print("Program terminated. Motors stopped.")