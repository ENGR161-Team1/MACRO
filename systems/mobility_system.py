import asyncio
import time
from buildhat import Motor


class MotionController:
    """
    Motor control with safety features and encoder tracking.
    
    Args:
        front_motor (str): Build HAT port for front motor (default: "A")
        turn_motor (str): Build HAT port for turn motor (default: "B")
        sensors (SensorInput): Centralized sensor manager for ultrasonic
        slowdown_distance (float): Distance to slow down in cm (default: 30.0)
        stopping_distance (float): Distance to stop in cm (default: 15.0)
        forward_speed (int): Normal forward speed (default: 20)
        forward_speed_slow (int): Slow forward speed (default: 10)
    """
    
    def __init__(self, **kwargs):
        self.front_motor = Motor(kwargs.get("front_motor", "A"))
        self.turn_motor = Motor(kwargs.get("turn_motor", "B"))
        
        # Centralized sensors
        self.sensors = kwargs.get("sensors", None)
        
        # Safety thresholds
        self.slowdown_distance = kwargs.get("slowdown_distance", 30.0)  # cm
        self.stopping_distance = kwargs.get("stopping_distance", 15.0)  # cm
        
        # Speed settings
        self.forward_speed = kwargs.get("forward_speed", 20)
        self.forward_speed_slow = kwargs.get("forward_speed_slow", 10)
        
        # State tracking
        self.moving = True
        self.current_speed = self.forward_speed
        
        # Motor encoder state
        self.motor_position = 0.0  # degrees
        self.motor_velocity = 0.0  # degrees per second
        self._prev_position = 0.0  # for velocity calculation

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

    def get_distance(self):
        """
        Get distance from ultrasonic sensor via SensorInput.
        
        Returns:
            float: Distance in cm, or default if no sensor
        """
        if self.sensors is None or not self.sensors.has_ultrasonic():
            return 30.0  # Default safe distance
        return self.sensors.get_distance()

    async def start_safety_ring(self):
        while True:
            dist = self.get_distance()
            
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

    async def update_motor_state(self, dt=0.1):
        """
        Update motor position and velocity from the encoder.
        
        Args:
            dt (float): Time step in seconds (default: 0.1)
        """
        self.motor_position = self.front_motor.get_position()
        
        if dt > 0:
            self.motor_velocity = (self.motor_position - self._prev_position) / dt
        
        self._prev_position = self.motor_position

    async def get_velocity(self):
        """
        Get the current motor velocity.
        
        Returns:
            float: Motor velocity in degrees per second
        """
        return self.motor_velocity

    def get_position(self):
        """
        Get the current motor position.
        
        Returns:
            float: Motor position in degrees
        """
        return self.motor_position

