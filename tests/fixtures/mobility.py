"""
Mobility fixture for MACRO tests.

Provides configurable MotionController setup.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from dataclasses import dataclass
from typing import Optional
from systems.mobility_system import MotionController
from systems.sensors import SensorInput


@dataclass
class MobilityConfig:
    """Configuration for MotionController setup."""
    # Motor ports
    front_motor: str = "A"
    turn_motor: str = "B"
    
    # Safety thresholds (cm)
    slowdown_distance: float = 30.0
    stopping_distance: float = 15.0
    
    # Speed settings
    forward_speed: int = 20
    forward_speed_slow: int = 10


def create_motion_controller(
    config: Optional[MobilityConfig] = None,
    sensors: Optional[SensorInput] = None
) -> MotionController:
    """
    Create a MotionController instance with the given configuration.
    
    Args:
        config: MobilityConfig instance (uses defaults if None)
        sensors: SensorInput instance (creates default if None)
    
    Returns:
        Configured MotionController instance
    """
    if config is None:
        config = MobilityConfig()
    
    if sensors is None:
        sensors = SensorInput(ultrasonic=True)
    
    motion = MotionController(
        front_motor=config.front_motor,
        turn_motor=config.turn_motor,
        sensors=sensors,
        slowdown_distance=config.slowdown_distance,
        stopping_distance=config.stopping_distance,
        forward_speed=config.forward_speed,
        forward_speed_slow=config.forward_speed_slow
    )
    
    return motion
