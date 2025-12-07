import asyncio
import time
from buildhat import Motor
from .state import State

class MotionController:
    """
    Motor control with safety features and encoder tracking.
    
    Args:
        front_motor (str): Build HAT port for front motor (default: "A")
        turn_motor (str): Build HAT port for turn motor (default: "B")
        state (State): Centralized state object for sensor values
        slowdown_distance (float): Distance to slow down in cm (default: 30.0)
        stopping_distance (float): Distance to stop in cm (default: 15.0)
        forward_speed (int): Normal forward speed (default: 20)
        forward_speed_slow (int): Slow forward speed (default: 10)
        turn_speed (int): Turn motor speed (default: 8)
        max_turn (int): Maximum turn angle in degrees (default: 100)
    """
    
    def __init__(self, **kwargs):
        self.front_motor = Motor(kwargs.get("front_motor", "A"))
        self.turn_motor = Motor(kwargs.get("turn_motor", "B"))
        
        # Centralized state
        self.state = kwargs.get("state", State())
        
        # Safety thresholds
        self.slowdown_distance = kwargs.get("slowdown_distance", 30.0)  # cm
        self.stopping_distance = kwargs.get("stopping_distance", 15.0)  # cm
        
        # Speed settings
        self.forward_speed = kwargs.get("forward_speed", 20)
        self.forward_speed_slow = kwargs.get("forward_speed_slow", 10)
        self.turn_speed = kwargs.get("turn_speed", 20)
        
        # Turn limits
        self.max_turn = kwargs.get("max_turn", 100)
        self.central_pos = self.turn_motor.get_position()
        
        # State tracking
        self.moving = False
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
    

    def get_distance(self) -> float:
        """
        Get distance from ultrasonic sensor via State.
        
        Returns:
            float: Distance in cm from state
        """
        return self.state.ultrasonic_distance

    async def start_safety_ring(self):
        self.moving = True
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

    async def update_motor_state(self):
        """Update motor encoder state."""
        self.state.motor_position = self.front_motor.get_position()
        self.state.motor_velocity = self.front_motor.get_speed()
        self.state.turn_position = self.turn_motor.get_position()

    async def increase_speed(self, increment: int = 5):
        """
        Increase forward speed.
        
        Args:
            increment (int): Amount to increase speed by (default: 5)
        """
        self.forward_speed += increment
        print(f"Increased forward speed to {self.forward_speed}")
    
    async def run_update_loop(self, update_interval: float = 0.1):
        """Continuously update motor state at specified intervals."""
        while True:
            await self.update_motor_state()
            await asyncio.sleep(update_interval)
    

    # --- Turning functions from POC ---
    
    async def turn_left_start(self):
        """Start turning left continuously."""
        self.turn_motor.stop()
        self.turn_motor.start(-self.turn_speed)

    async def turn_right_start(self):
        """Start turning right continuously."""
        self.turn_motor.stop()
        self.turn_motor.start(self.turn_speed)

    async def stop_turn(self):
        """Stop the turn motor."""
        self.turn_motor.stop()

    async def reverse(self):
        """Reverse the front motor."""
        self.front_motor.stop()
        self.front_motor.start(-self.forward_speed)
        self.moving = True
        self.current_speed = -self.forward_speed

    async def straighten(self):
        """Straighten wheels back to central position."""
        turn_pos = self.turn_motor.get_position()
        self.turn_motor.run_for_degrees(self.central_pos - turn_pos, self.turn_speed)

    async def turn_left(self, amount: int = 20):
        """
        Turn left by a specified amount if within limits.
        
        Args:
            amount (int): Degrees to turn (default: 20)
        """
        turn_pos = self.turn_motor.get_position()
        if turn_pos > self.central_pos - self.max_turn:
            self.turn_motor.run_for_degrees(-amount, self.turn_speed)

    async def turn_right(self, amount: int = 20):
        """
        Turn right by a specified amount if within limits.
        
        Args:
            amount (int): Degrees to turn (default: 20)
        """
        turn_pos = self.turn_motor.get_position()
        if turn_pos < self.central_pos + self.max_turn:
            self.turn_motor.run_for_degrees(amount, self.turn_speed)

    async def recalibrate_center(self):
        """Set current turn position as the new center."""
        self.central_pos = self.turn_motor.get_position()

    async def auto_line_follow(self):
        """
        Automatically follow a line using left and right line finders.
        Combines safety monitoring with line following.
        Reads line finder values and ultrasonic distance from State.
        """
        # Start forward motion
        self.front_motor.start(self.forward_speed)
        self.moving = True
        self.current_speed = self.forward_speed
        
        while True:
            # Safety check first
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
                    print("Path partially clear. Resuming at slow speed.")
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
            
            # Line following logic (only when moving)
            if self.moving:
                left_in = self.state.lf_left_value
                right_in = self.state.lf_right_value
                turn_pos = self.turn_motor.get_position()
                
                if left_in and right_in:
                    # Both sensors on line - correct based on current position
                    if turn_pos < self.central_pos:
                        await self.turn_left(20)
                    else:
                        await self.turn_right(20)
                elif left_in:
                    # Left sensor on line - turn left
                    await self.turn_left(20)
                elif right_in:
                    # Right sensor on line - turn right
                    await self.turn_right(20)
                else:
                    # Neither sensor on line - straighten
                    await self.straighten()
            
            await asyncio.sleep(0.1)
            

