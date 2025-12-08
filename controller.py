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
from systems.state import State
from systems.cargo_system import Cargo


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
    
    # Sensor offsets
    imu_height: float = 0.015  # Height of IMU from ground in meters
    color_sensor_height: float = 0.025  # Height of color sensor from ground in meters
    lf_height: float = 0.025  # Height of left front distance sensor from ground in meters
    lf_offset: float = 0.0225 # Horizontal offset of left front distance sensor from center in meters
    imu_to_lf: float = 0.125 # Horizontal distance from IMU to front distance sensors in meters
    imu_to_color: float = 0.11 # Horizontal distance from IMU to color sensor in meters
    imu_to_cargo: float = 0.24 # Horizontal distance from IMU to cargo deploy location in meters
    
    # Update interval
    update_interval: float = 0.05


@dataclass
class MobilityConfig:
    """Mobility configuration from [mobility] section."""
    front_motor: str = "A"
    turn_motor: str = "B"
    
    # Speed
    forward_speed: int = 20
    forward_speed_slow: int = 10
    turn_speed: int = 20
    
    # Turn limits
    max_turn: int = 100
    
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
    print_state: bool = False
    print_fields: List[str] = field(default_factory=lambda: ["all"])


@dataclass
class DisplayConfig:
    """Display configuration from [display] section.
    
    Note: For future UI integration (navigation_display.py).
    """
    enabled: bool = True
    update_interval: int = 100  # milliseconds
    
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
    """Testing configuration from [testing] section.
    
    Note: For future test framework integration.
    """
    mock_hardware: bool = False
    verbose: bool = True
    
    # Timeouts
    calibration_timeout: float = 5.0
    navigation_update_timeout: float = 1.0
    motor_response_timeout: float = 2.0


@dataclass
class CargoConfig:
    """Cargo detection configuration from [cargo] section."""
    # Motor settings
    motor_port: str = "B"
    motor_speed: int = 50
    deploy_angle: int = 180
    
    # Magnetic thresholds in micro-tesla
    edge_threshold: float = 400.0
    semi_threshold: float = 1000.0
    full_threshold: float = 3000.0
    
    # Debounce: consecutive detections needed before deploying
    required_detections: int = 5


@dataclass
class Config:
    """Complete MACRO configuration."""
    sensors: SensorConfig = field(default_factory=SensorConfig)
    mobility: MobilityConfig = field(default_factory=MobilityConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    cargo: CargoConfig = field(default_factory=CargoConfig)
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
            imu_height=s.get("imu_height", 0.015),
            color_sensor_height=s.get("color_sensor_height", 0.025),
            lf_height=s.get("lf_height", 0.025),
            lf_offset=s.get("lf_offset", 0.0225),
            imu_to_lf=s.get("imu_to_lf", 0.125),
            imu_to_color=s.get("imu_to_color", 0.11),
            imu_to_cargo=s.get("imu_to_cargo", 0.24),
            update_interval=s.get("update_interval", 0.05),
        )
    
    # Parse mobility section
    if "mobility" in data:
        m = data["mobility"]
        config.mobility = MobilityConfig(
            front_motor=m.get("front_motor", "A"),
            turn_motor=m.get("turn_motor", "B"),
            forward_speed=m.get("speed", {}).get("forward", 20),
            forward_speed_slow=m.get("speed", {}).get("forward_slow", 10),
            turn_speed=m.get("speed", {}).get("turn", 20),
            max_turn=m.get("turn", {}).get("max_angle", 100),
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
    
    # Parse cargo section
    if "cargo" in data:
        c = data["cargo"]
        config.cargo = CargoConfig(
            motor_port=c.get("motor", {}).get("port", "B"),
            motor_speed=c.get("motor", {}).get("speed", 50),
            deploy_angle=c.get("motor", {}).get("deploy_angle", 180),
            edge_threshold=c.get("thresholds", {}).get("edge", 400.0),
            semi_threshold=c.get("thresholds", {}).get("semi", 1000.0),
            full_threshold=c.get("thresholds", {}).get("full", 3000.0),
            required_detections=c.get("detection", {}).get("required_consecutive", 5),
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
    Uses a shared State instance across all systems.
    
    Args:
        config_path: Optional path to config file
    
    Attributes:
        config: Parsed configuration
        state: Shared State instance
        sensors: SensorInput instance
        mobility: MotionController instance
        navigator: Navigation instance
        cargo: Cargo instance
    """
    
    def __init__(self, config_path: Optional[str] = None):
        self.config = load_config(config_path)
        self.state: State = State()
        self.sensors: Optional[SensorInput] = None
        self.mobility: Optional[MotionController] = None
        self.navigator: Optional[Navigation] = None
        self.cargo: Optional[Cargo] = None
        self._running = False
        self._sensor_task = None
        self._cargo_monitor_task = None
        self._motor_monitor_task = None
    
    async def initialize(self):
        """Initialize all systems based on configuration."""
        sc = self.config.sensors
        nc = self.config.navigation
        mc = self.config.mobility
        
        # Initialize sensors with shared state
        self.sensors = SensorInput(
            state=self.state,
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
        
        # Initialize mobility with shared state
        self.mobility = MotionController(
            state=self.state,
            front_motor=mc.front_motor,
            turn_motor=mc.turn_motor,
            slowdown_distance=mc.slowdown_distance,
            stopping_distance=mc.stopping_distance,
            forward_speed=mc.forward_speed,
            forward_speed_slow=mc.forward_speed_slow,
            turn_speed=mc.turn_speed,
            max_turn=mc.max_turn,
        )
        
        # Initialize navigation with shared state
        self.navigator = Navigation(
            state=self.state,
            position=nc.position,
            orientation=nc.orientation,
            mode=nc.mode,
            velocity_decay=nc.velocity_decay,
            accel_threshold=nc.accel_threshold,
            motor_velocity_threshold=nc.motor_velocity_threshold,
            motion_controller=self.mobility,
            imu_height=sc.imu_height,
            color_sensor_height=sc.color_sensor_height,
            lf_height=sc.lf_height,
            lf_offset=sc.lf_offset,
            imu_to_lf=sc.imu_to_lf,
            imu_to_color=sc.imu_to_color,
            imu_to_cargo=sc.imu_to_cargo,
        )
        
        # Initialize cargo with shared state
        cc = self.config.cargo
        self.cargo = Cargo(
            state=self.state,
            motor_port=cc.motor_port,
            motor_speed=cc.motor_speed,
            deploy_angle=cc.deploy_angle,
            edge_threshold=cc.edge_threshold,
            semi_threshold=cc.semi_threshold,
            full_threshold=cc.full_threshold,
            required_detections=cc.required_detections,
        )
        
        # Start sensor update loop
        self._sensor_task = asyncio.create_task(
            self.sensors.run_sensor_update(update_interval=sc.update_interval)
        )
        
        # Allow sensors to start
        await asyncio.sleep(0.1)
        
        # Calibrate if enabled
        if nc.calibrate:
            await self.sensors.calibrate_imu(
                samples=nc.calibration_samples,
                delay=nc.calibration_delay,
            )

        # Start motor state update loop
        self._motor_monitor_task = asyncio.create_task(
            self.mobility.run_update_loop(update_interval=sc.update_interval)
        )

        print("MACRO Controller initialized")
    
    def print_state(self, timestamp: float, fields: List[str] = None):
        """
        Print the current state with timestamp.
        
        Args:
            timestamp (float): Current timestamp in seconds since start
            fields (list): List of fields to print. Options:
                - "position": Robot position (x, y, z)
                - "velocity": Robot velocity
                - "acceleration": Robot acceleration
                - "orientation": Robot orientation (yaw, pitch, roll)
                - "magnetic": Magnetic field magnitude
                - "ultrasonic": Ultrasonic distance
                - "line_finder": Left and right line finder values
                - "motor": Front motor position and velocity
                - "turn": Turn motor position
                - "all": All fields
                Use [] or None for no output. Default: ["all"]
        """
        if fields is None:
            fields = ["all"]
        
        if not fields:
            return
        
        show_all = "all" in fields
        parts = [f"[{timestamp:.3f}s]"]
        
        if show_all or "position" in fields:
            pos = tuple(round(p, 3) for p in self.state.position)
            parts.append(f"Pos: {pos}")
        
        if show_all or "velocity" in fields:
            vel = tuple(round(v, 3) for v in self.state.velocity)
            parts.append(f"Vel: {vel}")
        
        if show_all or "acceleration" in fields:
            acc = tuple(round(a, 3) for a in self.state.acceleration)
            parts.append(f"Acc: {acc}")
        
        if show_all or "orientation" in fields:
            orient = tuple(round(o, 2) for o in self.state.orientation)
            parts.append(f"Orient: {orient}")
        
        if show_all or "magnetic" in fields:
            parts.append(f"Mag: {self.state.magnetic_field:.2f} µT Adjusted: {self.state.mag_delta:.2f} µT")

        if show_all or "cargo" in fields:
            parts.append(f"Cargo: {self.state.cargo_level}")
        
        if show_all or "ultrasonic" in fields:
            parts.append(f"Dist: {self.state.ultrasonic_distance:.1f} cm")
        
        if show_all or "line_finder" in fields:
            parts.append(f"LF: L={self.state.lf_left_value:.0f} R={self.state.lf_right_value:.0f}")
        
        if show_all or "motor" in fields:
            parts.append(f"Motor: pos={self.state.motor_position:.1f}° vel={self.state.motor_velocity:.1f}°/s")
        
        if show_all or "turn" in fields:
            parts.append(f"Turn: {self.state.turn_position:.1f}°")
        
        if len(parts) > 1:
            print(", ".join(parts))
    
    async def run(self):
        """Run the main control loop with line following."""
        import time
        
        self._running = True
        nc = self.config.navigation
        start_time = time.time()
        
        print("MACRO Controller running...")
        
        # Start line following task
        line_follow_task = asyncio.create_task(self.mobility.auto_line_follow())

        # Start cargo monitoring task
        cargo_monitor_task = asyncio.create_task(
            self.cargo.run_cargo_update_loop(update_interval=self.config.sensors.update_interval)
        )
        
        try:
            while self._running:
                await self.navigator.update_state(dt=nc.update_interval)
                
                if nc.print_state:
                    timestamp = time.time() - start_time
                    self.print_state(timestamp, nc.print_fields)
                
                await asyncio.sleep(nc.update_interval)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received...")
        finally:
            line_follow_task.cancel()
            await self.shutdown()
    
    async def shutdown(self):
        """
        Graceful shutdown sequence:
        1. Straighten the wheels
        2. Stop all data collection
        3. Stop all motors
        """
        print("Shutting down MACRO Controller...")
        
        # 1. Straighten the wheels
        if self.mobility:
            print("Straightening wheels...")
            await self.mobility.straighten()
        
        # 2. Stop all data collection (sensor update loop)
        self._running = False
        if self._sensor_task:
            self._sensor_task.cancel()
            try:
                await self._sensor_task
            except asyncio.CancelledError:
                pass
        
        # 3. Stop all motors
        if self.mobility:
            print("Stopping motors...")
            self.mobility.stop()
        
        print("MACRO Controller stopped")
    
    def stop(self):
        """Stop all systems (synchronous version for emergency stops)."""
        self._running = False
        if self._sensor_task:
            self._sensor_task.cancel()
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

