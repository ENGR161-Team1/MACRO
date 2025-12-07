"""
controller.py

Central controller for MACRO that reads macro_config.toml and initializes all systems.

This module provides:
- Config: Dataclass for parsed configuration
- Controller: Main controller that initializes and manages all systems

Usage:
    from controller import Controller
    
    controller = Controller()  # Uses default macro_config.toml
    await controller.initialize()
    await controller.run()
"""

import asyncio
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Dict, Any

try:
    import tomllib  # Python 3.11+
except ImportError:
    import tomli as tomllib  # Fallback for Python 3.10

from systems.sensors import SensorInput
from systems.mobility_system import MotionController
from systems.navigation_system import Navigation


@dataclass
class SensorConfig:
    """Sensor configuration from [sensors] section."""
    imu: bool = True
    ultrasonic: bool = True
    line_finders: bool = False
    button: bool = False
    color_sensor: bool = False
    
    # Pins
    ultrasonic_pin: int = 26
    line_finder_left_pin: int = 16
    line_finder_right_pin: int = 5
    button_pin: int = 22
    
    # Ports
    color_sensor_port: str = "D"


@dataclass
class MobilityConfig:
    """Mobility configuration from [mobility] section."""
    front_motor: str = "A"
    turn_motor: str = "B"
    
    # Speed
    forward_speed: int = 20
    forward_speed_slow: int = 10
    reverse_speed: int = -20
    
    # Safety
    slowdown_distance: float = 30.0
    stopping_distance: float = 15.0


@dataclass
class NavigationConfig:
    """Navigation configuration from [navigation] section."""
    mode: str = "degrees"
    
    # Initial state
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    
    # Calibration
    calibrate: bool = True
    calibration_samples: int = 50
    calibration_delay: float = 0.02
    
    # Drift reduction
    velocity_decay: float = 0.04
    accel_threshold: float = 0.05
    motor_velocity_threshold: float = 1.0
    
    # Update
    update_interval: float = 0.1
    log_state: bool = True
    print_state: bool = False
    print_fields: List[str] = field(default_factory=lambda: ["all"])


@dataclass
class DisplayConfig:
    """Display configuration from [display] section."""
    enabled: bool = True
    update_interval: int = 100
    
    # Window
    width: int = 800
    height: int = 600
    title: str = "MACRO Control Panel"
    
    # Colors
    background: str = "#1a1a2e"
    foreground: str = "#eaeaea"
    accent: str = "#e94560"
    success: str = "#4ecca3"
    warning: str = "#ffc107"


@dataclass
class TestingConfig:
    """Testing configuration from [testing] section."""
    mock_hardware: bool = False
    verbose: bool = True
    
    # Timeouts
    calibration_timeout: float = 5.0
    navigation_update_timeout: float = 1.0
    motor_response_timeout: float = 2.0


@dataclass
class Config:
    """Complete MACRO configuration."""
    sensors: SensorConfig = field(default_factory=SensorConfig)
    mobility: MobilityConfig = field(default_factory=MobilityConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    display: DisplayConfig = field(default_factory=DisplayConfig)
    testing: TestingConfig = field(default_factory=TestingConfig)


def load_config(config_path: Optional[str] = None) -> Config:
    """
    Load configuration from TOML file.
    
    Args:
        config_path: Path to config file (default: macro_config.toml in project root)
    
    Returns:
        Parsed Config dataclass
    """
    if config_path is None:
        config_path = Path(__file__).parent / "macro_config.toml"
    else:
        config_path = Path(config_path)
    
    with open(config_path, "rb") as f:
        data = tomllib.load(f)
    
    config = Config()
    
    # Parse sensors section
    if "sensors" in data:
        s = data["sensors"]
        config.sensors = SensorConfig(
            imu=s.get("imu", True),
            ultrasonic=s.get("ultrasonic", True),
            line_finders=s.get("line_finders", False),
            button=s.get("button", False),
            color_sensor=s.get("color_sensor", False),
            ultrasonic_pin=s.get("pins", {}).get("ultrasonic", 26),
            line_finder_left_pin=s.get("pins", {}).get("line_finder_left", 16),
            line_finder_right_pin=s.get("pins", {}).get("line_finder_right", 5),
            button_pin=s.get("pins", {}).get("button", 22),
            color_sensor_port=s.get("ports", {}).get("color_sensor", "D"),
        )
    
    # Parse mobility section
    if "mobility" in data:
        m = data["mobility"]
        config.mobility = MobilityConfig(
            front_motor=m.get("front_motor", "A"),
            turn_motor=m.get("turn_motor", "B"),
            forward_speed=m.get("speed", {}).get("forward", 20),
            forward_speed_slow=m.get("speed", {}).get("forward_slow", 10),
            reverse_speed=m.get("speed", {}).get("reverse", -20),
            slowdown_distance=m.get("safety", {}).get("slowdown_distance", 30.0),
            stopping_distance=m.get("safety", {}).get("stopping_distance", 15.0),
        )
    
    # Parse navigation section
    if "navigation" in data:
        n = data["navigation"]
        config.navigation = NavigationConfig(
            mode=n.get("mode", "degrees"),
            position=n.get("initial", {}).get("position", [0.0, 0.0, 0.0]),
            orientation=n.get("initial", {}).get("orientation", [0.0, 0.0, 0.0]),
            calibrate=n.get("calibration", {}).get("enabled", True),
            calibration_samples=n.get("calibration", {}).get("samples", 50),
            calibration_delay=n.get("calibration", {}).get("delay", 0.02),
            velocity_decay=n.get("drift_reduction", {}).get("velocity_decay", 0.04),
            accel_threshold=n.get("drift_reduction", {}).get("accel_threshold", 0.05),
            motor_velocity_threshold=n.get("drift_reduction", {}).get("motor_velocity_threshold", 1.0),
            update_interval=n.get("update", {}).get("interval", 0.1),
            log_state=n.get("update", {}).get("log_state", True),
            print_state=n.get("update", {}).get("print_state", False),
            print_fields=n.get("update", {}).get("print_fields", ["all"]),
        )
    
    # Parse display section
    if "display" in data:
        d = data["display"]
        config.display = DisplayConfig(
            enabled=d.get("enabled", True),
            update_interval=d.get("update_interval", 100),
            width=d.get("window", {}).get("width", 800),
            height=d.get("window", {}).get("height", 600),
            title=d.get("window", {}).get("title", "MACRO Control Panel"),
            background=d.get("colors", {}).get("background", "#1a1a2e"),
            foreground=d.get("colors", {}).get("foreground", "#eaeaea"),
            accent=d.get("colors", {}).get("accent", "#e94560"),
            success=d.get("colors", {}).get("success", "#4ecca3"),
            warning=d.get("colors", {}).get("warning", "#ffc107"),
        )
    
    # Parse testing section
    if "testing" in data:
        te = data["testing"]
        config.testing = TestingConfig(
            mock_hardware=te.get("mock_hardware", False),
            verbose=te.get("verbose", True),
            calibration_timeout=te.get("timeouts", {}).get("calibration", 5.0),
            navigation_update_timeout=te.get("timeouts", {}).get("navigation_update", 1.0),
            motor_response_timeout=te.get("timeouts", {}).get("motor_response", 2.0),
        )
    
    return config


class Controller:
    """
    Central controller for MACRO.
    
    Reads configuration from macro_config.toml and initializes all systems.
    Provides unified access to sensors, mobility, navigation, and other systems.
    
    Args:
        config_path: Optional path to config file
    
    Attributes:
        config: Parsed configuration
        sensors: SensorInput instance
        mobility: MotionController instance
        navigator: Navigation instance
    """
    
    def __init__(self, config_path: Optional[str] = None):
        self.config = load_config(config_path)
        self.sensors: Optional[SensorInput] = None
        self.mobility: Optional[MotionController] = None
        self.navigator: Optional[Navigation] = None
        self._running = False
    
    async def initialize(self):
        """Initialize all systems based on configuration."""
        # Initialize sensors
        sc = self.config.sensors
        self.sensors = SensorInput(
            imu=sc.imu,
            ultrasonic=sc.ultrasonic,
            ultrasonic_pin=sc.ultrasonic_pin,
            line_finders=sc.line_finders,
            line_finder_left_pin=sc.line_finder_left_pin,
            line_finder_right_pin=sc.line_finder_right_pin,
            button=sc.button,
            button_pin=sc.button_pin,
            color_sensor=sc.color_sensor,
            color_sensor_port=sc.color_sensor_port,
        )
        
        # Initialize mobility
        mc = self.config.mobility
        self.mobility = MotionController(
            front_motor=mc.front_motor,
            turn_motor=mc.turn_motor,
            sensors=self.sensors,
            slowdown_distance=mc.slowdown_distance,
            stopping_distance=mc.stopping_distance,
            forward_speed=mc.forward_speed,
            forward_speed_slow=mc.forward_speed_slow,
        )
        
        # Initialize navigation
        nc = self.config.navigation
        self.navigator = Navigation(
            sensors=self.sensors,
            position=nc.position,
            orientation=nc.orientation,
            mode=nc.mode,
            velocity_decay=nc.velocity_decay,
            accel_threshold=nc.accel_threshold,
            motor_velocity_threshold=nc.motor_velocity_threshold,
            motion_controller=self.mobility,
        )
        
        # Calibrate if enabled
        if nc.calibrate:
            await self.navigator.calibrate(
                samples=nc.calibration_samples,
                delay=nc.calibration_delay,
            )
        
        print("MACRO Controller initialized")
    
    async def run(self):
        """Run the main control loop."""
        self._running = True
        nc = self.config.navigation
        
        print("MACRO Controller running...")
        
        try:
            while self._running:
                await self.navigator.update_state(dt=nc.update_interval)
                
                if nc.log_state:
                    self.navigator.log_state(asyncio.get_event_loop().time())
                
                await asyncio.sleep(nc.update_interval)
        except KeyboardInterrupt:
            print("\nStopping MACRO Controller...")
        finally:
            self.stop()
    
    def stop(self):
        """Stop all systems."""
        self._running = False
        if self.mobility:
            self.mobility.stop()
        print("MACRO Controller stopped")


# Convenience function for quick access
def get_config(config_path: Optional[str] = None) -> Config:
    """Load and return configuration."""
    return load_config(config_path)


if __name__ == "__main__":
    async def main():
        controller = Controller()
        await controller.initialize()
        await controller.run()
    
    asyncio.run(main())

