"""
Sensor fixture for MACRO tests.

Provides configurable SensorInput setup.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from dataclasses import dataclass
from typing import Optional
from systems.sensors import SensorInput


@dataclass
class SensorConfig:
    """Configuration for SensorInput setup."""
    # IMU
    imu: bool = True
    
    # Ultrasonic
    ultrasonic: bool = True
    ultrasonic_pin: int = 26
    
    # Line finders (disabled by default)
    line_finders: bool = False
    line_finder_left_pin: int = 16
    line_finder_right_pin: int = 5
    
    # Button (disabled by default)
    button: bool = False
    button_pin: int = 22
    
    # Hall sensor (disabled by default)
    hall: bool = False
    hall_pin: int = 12


def create_sensors(config: Optional[SensorConfig] = None) -> SensorInput:
    """
    Create a SensorInput instance with the given configuration.
    
    Args:
        config: SensorConfig instance (uses defaults if None)
    
    Returns:
        Configured SensorInput instance
    """
    if config is None:
        config = SensorConfig()
    
    sensors = SensorInput(
        imu=config.imu,
        ultrasonic=config.ultrasonic,
        ultrasonic_pin=config.ultrasonic_pin,
        line_finders=config.line_finders,
        line_finder_left_pin=config.line_finder_left_pin,
        line_finder_right_pin=config.line_finder_right_pin,
        button=config.button,
        button_pin=config.button_pin,
        hall=config.hall,
        hall_pin=config.hall_pin
    )
    
    return sensors
