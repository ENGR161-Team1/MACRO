"""
Mobility fixture for MARCO tests.

Provides configurable MotionController setup.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from dataclasses import dataclass
from typing import Optional
from systems.mobility_system import MotionController


@dataclass
class MobilityConfig:
    """Configuration for MotionController setup."""
    # Motor ports
    front_motor: str = "A"
    turn_motor: str = "B"
    
    # Ultrasonic sensor
    ultrasonic_pin: int = 26
    
    # Safety thresholds (cm)
    slowdown_distance: float = 30.0
    stopping_distance: float = 15.0
    
    # Speed settings
    forward_speed: int = 20
    forward_speed_slow: int = 10


def create_motion_controller(config: Optional[MobilityConfig] = None) -> MotionController:
    """
    Create a MotionController instance with the given configuration.
    
    Args:
        config: MobilityConfig instance (uses defaults if None)
    
    Returns:
        Configured MotionController instance
    """
    if config is None:
        config = MobilityConfig()
    
    motion = MotionController(
        front_motor=config.front_motor,
        turn_motor=config.turn_motor,
        ultrasonic_pin=config.ultrasonic_pin,
        slowdown_distance=config.slowdown_distance,
        stopping_distance=config.stopping_distance,
        forward_speed=config.forward_speed,
        forward_speed_slow=config.forward_speed_slow
    )
    
    return motion
