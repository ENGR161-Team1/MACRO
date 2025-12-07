"""
Navigation fixture for MACRO tests.

Provides configurable Navigation3D setup.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from dataclasses import dataclass, field
from typing import List, Optional
from systems.navigation_system import Navigation3D
from systems.sensors import SensorInput


@dataclass
class NavigationConfig:
    """Configuration for Navigation3D setup."""
    # Update settings
    update_interval: float = 0.1
    log_state: bool = True
    print_state: bool = False
    print_fields: List[str] = field(default_factory=lambda: ["all"])
    
    # Calibration
    calibrate: bool = True
    calibration_samples: int = 50
    
    # Drift reduction
    velocity_decay: float = 0.04
    accel_threshold: float = 0.05
    motor_velocity_threshold: float = 1.0
    
    # Initial state
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    
    # Mode
    mode: str = "degrees"


def create_navigator(
    config: Optional[NavigationConfig] = None,
    sensors: Optional[SensorInput] = None,
    motion_controller=None
) -> Navigation3D:
    """
    Create a Navigation3D instance with the given configuration.
    
    Args:
        config: NavigationConfig instance (uses defaults if None)
        sensors: SensorInput instance (creates default if None)
        motion_controller: Optional MotionController for motor velocity tracking
    
    Returns:
        Configured Navigation3D instance
    """
    if config is None:
        config = NavigationConfig()
    
    if sensors is None:
        sensors = SensorInput(imu=True)
    
    navigator = Navigation3D(
        sensors=sensors,
        position=config.position,
        orientation=config.orientation,
        mode=config.mode,
        velocity_decay=config.velocity_decay,
        accel_threshold=config.accel_threshold,
        motor_velocity_threshold=config.motor_velocity_threshold,
        motion_controller=motion_controller
    )
    
    return navigator


async def run_navigation(navigator: Navigation3D, config: Optional[NavigationConfig] = None):
    """
    Run continuous navigation updates with the given configuration.
    
    Args:
        navigator: Navigation3D instance
        config: NavigationConfig instance (uses defaults if None)
    """
    if config is None:
        config = NavigationConfig()
    
    await navigator.run_continuous_update(
        update_interval=config.update_interval,
        log_state=config.log_state,
        print_state=config.print_state,
        calibrate=config.calibrate,
        calibration_samples=config.calibration_samples
    )
