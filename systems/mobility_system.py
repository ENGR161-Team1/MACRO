import asyncio
from buildhat import Motor
from basehat import UltrasonicSensor

class MotionController:
  def __init__(self, **kwargs):
    self.front_motor = Motor(kwargs.get("front_motor", "A"))
    self.turn_motor = Motor(kwargs.get("turn_motor", "B"))
    
    # Ultrasonic sensor setup
    ultrasonic_pin = kwargs.get("ultrasonic_pin", 26)
    self.ultrasonic_sensor = UltrasonicSensor(ultrasonic_pin)
    
    # Safety thresholds
    self.slowdown_distance = kwargs.get("slowdown_distance", 30.0)  # cm
    self.stopping_distance = kwargs.get("stopping_distance", 15.0)  # cm
    
    # Speed settings
    self.forward_speed = kwargs.get("forward_speed", 20)
    self.forward_speed_slow = kwargs.get("forward_speed_slow", 10)
    
    # State tracking
    self.moving = True
    self.current_speed = self.forward_speed

  def start(self, speed=None):
    if speed is None:
      speed = self.forward_speed
    self.front_motor.start(speed)
    self.moving = True
    self.current_speed = speed

  def stop(self):
    self.front_motor.stop()
    self.turn_motor.stop()
    self.moving = False

  async def start_safety_ring(self):
    while True:
      try:
        dist = float(self.ultrasonic_sensor.getDist)
      except:
        dist = 30.0
      
      if dist < self.stopping_distance:
        if self.moving:
          self.front_motor.stop()
          print("Obstacle detected! Stopping motors.")
          self.moving = False
      elif dist < self.slowdown_distance:
        if self.moving and self.current_speed != self.forward_speed_slow:
          self.front_motor.start(self.forward_speed_slow)
          print("Obstacle nearby! Slowing down.")
          self.current_speed = self.forward_speed_slow
        elif not self.moving:
          self.front_motor.start(self.forward_speed_slow)
          print("Path partially clear. Resuming movement at slow speed.")
          self.moving = True
          self.current_speed = self.forward_speed_slow
      else:
        if not self.moving:
          self.front_motor.start(self.forward_speed)
          print("Path clear. Resuming movement.")
          self.moving = True
          self.current_speed = self.forward_speed
        elif self.current_speed != self.forward_speed:
          self.front_motor.start(self.forward_speed)
          print("Path clear. Speeding up.")
          self.current_speed = self.forward_speed
      
      await asyncio.sleep(0.1)

  async def run_with_safety(self):
    """Start moving and run the safety ring."""
    self.front_motor.start(self.forward_speed)
    await self.start_safety_ring()

