"""
test.py

Unified test runner for MACRO.

Reads test_config.toml and runs the appropriate combination of systems
based on enabled features.

Usage:
    python tests/test.py                    # Use default test_config.toml
    python tests/test.py path/to/config.toml  # Use custom config
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncio
from dataclasses import dataclass, field
from typing import List, Optional

try:
    import tomllib
except ImportError:
    import tomli as tomllib

from buildhat import Motor
from systems.sensors import SensorInput
from systems.mobility_system import MotionController
from systems.navigation_system import Navigation
from ui.navigation_display import NavigationDisplay


# =============================================================================
# CONFIGURATION DATACLASSES
# =============================================================================

@dataclass
class FeaturesConfig:
    """Feature toggles."""
    navigation: bool = True
    mobility: bool = True
    display: bool = True
    safety_ring: bool = True
    manual_control: bool = False
    autonomous: bool = True
    payload: bool = False
    color_sensor: bool = False
    magnetism: bool = False


@dataclass
class SensorConfig:
    """Sensor configuration."""
    imu: bool = True
    ultrasonic: bool = True
    line_finders: bool = False
    button: bool = False
    color_sensor: bool = False
    ultrasonic_pin: int = 26
    line_finder_left_pin: int = 16
    line_finder_right_pin: int = 5
    button_pin: int = 22
    color_sensor_port: str = "D"


@dataclass
class NavigationConfig:
    """Navigation configuration."""
    update_interval: float = 0.1
    log_state: bool = True
    print_state: bool = False
    print_fields: List[str] = field(default_factory=lambda: ["position", "velocity"])
    calibrate: bool = True
    calibration_samples: int = 50
    calibration_delay: float = 0.02
    velocity_decay: float = 0.04
    accel_threshold: float = 0.05
    motor_velocity_threshold: float = 1.0
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class MobilityConfig:
    """Mobility configuration."""
    front_motor: str = "A"
    turn_motor: str = "B"
    forward_speed: int = 20
    forward_speed_slow: int = 10
    reverse_speed: int = -20
    slowdown_distance: float = 30.0
    stopping_distance: float = 15.0


@dataclass
class DisplayConfig:
    """Display configuration."""
    width: int = 800
    height: int = 800
    scale: float = 50.0
    world_min: float = -10.0
    world_max: float = 10.0


@dataclass
class ManualControlConfig:
    """Manual control configuration."""
    turn_amount: int = 45
    max_turn: int = 100


@dataclass
class PayloadConfig:
    """Payload configuration."""
    motor: str = "C"
    speed: int = 2


@dataclass
class LoggingConfig:
    """Logging configuration."""
    enabled: bool = True
    file: str = "logs/test_run.log"
    format: str = "csv"  # "csv", "json", "text"
    log_position: bool = True
    log_velocity: bool = True
    log_orientation: bool = True
    log_acceleration: bool = True
    log_magnetic_field: bool = False
    log_motor_velocity: bool = True
    log_distance: bool = False
    timestamp_format: str = "elapsed"  # "elapsed", "absolute", "iso"


@dataclass
class ConsoleConfig:
    """Console output configuration."""
    enabled: bool = False
    interval: float = 0.2
    print_position: bool = True
    print_velocity: bool = True
    print_orientation: bool = False
    print_acceleration: bool = False
    print_magnetic_field: bool = False
    print_motor_velocity: bool = False
    print_distance: bool = False
    format: str = "compact"  # "compact" or "detailed"
    precision: int = 3


@dataclass
class ReadingsConfig:
    """Sensor readings configuration."""
    imu_interval: float = 0.1
    imu_average_samples: int = 1
    ultrasonic_interval: float = 0.1
    ultrasonic_average_samples: int = 3
    color_interval: float = 0.2
    magnetic_interval: float = 0.2


@dataclass 
class TestConfig:
    """Complete test configuration."""
    features: FeaturesConfig = field(default_factory=FeaturesConfig)
    sensors: SensorConfig = field(default_factory=SensorConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    mobility: MobilityConfig = field(default_factory=MobilityConfig)
    display: DisplayConfig = field(default_factory=DisplayConfig)
    manual_control: ManualControlConfig = field(default_factory=ManualControlConfig)
    payload: PayloadConfig = field(default_factory=PayloadConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    console: ConsoleConfig = field(default_factory=ConsoleConfig)
    readings: ReadingsConfig = field(default_factory=ReadingsConfig)
    duration: float = 0
    verbose: bool = True


def load_test_config(config_path: Optional[str] = None) -> TestConfig:
    """Load test configuration from TOML file."""
    if config_path is None:
        config_path = Path(__file__).parent / "test_config.toml"
    else:
        config_path = Path(config_path)
    
    with open(config_path, "rb") as f:
        data = tomllib.load(f)
    
    config = TestConfig()
    
    # Parse features
    if "features" in data:
        f = data["features"]
        config.features = FeaturesConfig(
            navigation=f.get("navigation", True),
            mobility=f.get("mobility", True),
            display=f.get("display", True),
            safety_ring=f.get("safety_ring", True),
            manual_control=f.get("manual_control", False),
            autonomous=f.get("autonomous", True),
            payload=f.get("payload", False),
            color_sensor=f.get("color_sensor", False),
            magnetism=f.get("magnetism", False),
        )
    
    # Parse sensors
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
    
    # Parse navigation
    if "navigation" in data:
        n = data["navigation"]
        config.navigation = NavigationConfig(
            update_interval=n.get("update_interval", 0.1),
            log_state=n.get("log_state", True),
            print_state=n.get("print_state", False),
            print_fields=n.get("print_fields", ["position", "velocity"]),
            calibrate=n.get("calibration", {}).get("enabled", True),
            calibration_samples=n.get("calibration", {}).get("samples", 50),
            calibration_delay=n.get("calibration", {}).get("delay", 0.02),
            velocity_decay=n.get("drift_reduction", {}).get("velocity_decay", 0.04),
            accel_threshold=n.get("drift_reduction", {}).get("accel_threshold", 0.05),
            motor_velocity_threshold=n.get("drift_reduction", {}).get("motor_velocity_threshold", 1.0),
            position=n.get("initial", {}).get("position", [0.0, 0.0, 0.0]),
            orientation=n.get("initial", {}).get("orientation", [0.0, 0.0, 0.0]),
        )
    
    # Parse mobility
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
    
    # Parse display
    if "display" in data:
        d = data["display"]
        config.display = DisplayConfig(
            width=d.get("width", 800),
            height=d.get("height", 800),
            scale=d.get("scale", 50.0),
            world_min=d.get("world", {}).get("min", -10.0),
            world_max=d.get("world", {}).get("max", 10.0),
        )
    
    # Parse manual control
    if "manual_control" in data:
        mc = data["manual_control"]
        config.manual_control = ManualControlConfig(
            turn_amount=mc.get("turn_amount", 45),
            max_turn=mc.get("max_turn", 100),
        )
    
    # Parse payload
    if "payload" in data:
        p = data["payload"]
        config.payload = PayloadConfig(
            motor=p.get("motor", "C"),
            speed=p.get("speed", 2),
        )
    
    # Parse logging
    if "logging" in data:
        lg = data["logging"]
        config.logging = LoggingConfig(
            enabled=lg.get("enabled", True),
            file=lg.get("file", "logs/test_run.log"),
            format=lg.get("format", "csv"),
            log_position=lg.get("log_position", True),
            log_velocity=lg.get("log_velocity", True),
            log_orientation=lg.get("log_orientation", True),
            log_acceleration=lg.get("log_acceleration", True),
            log_magnetic_field=lg.get("log_magnetic_field", False),
            log_motor_velocity=lg.get("log_motor_velocity", True),
            log_distance=lg.get("log_distance", False),
            timestamp_format=lg.get("timestamp_format", "elapsed"),
        )
    
    # Parse console
    if "console" in data:
        c = data["console"]
        config.console = ConsoleConfig(
            enabled=c.get("enabled", False),
            interval=c.get("interval", 0.2),
            print_position=c.get("print_position", True),
            print_velocity=c.get("print_velocity", True),
            print_orientation=c.get("print_orientation", False),
            print_acceleration=c.get("print_acceleration", False),
            print_magnetic_field=c.get("print_magnetic_field", False),
            print_motor_velocity=c.get("print_motor_velocity", False),
            print_distance=c.get("print_distance", False),
            format=c.get("format", "compact"),
            precision=c.get("precision", 3),
        )
    
    # Parse readings
    if "readings" in data:
        r = data["readings"]
        config.readings = ReadingsConfig(
            imu_interval=r.get("imu_interval", 0.1),
            imu_average_samples=r.get("imu_average_samples", 1),
            ultrasonic_interval=r.get("ultrasonic_interval", 0.1),
            ultrasonic_average_samples=r.get("ultrasonic_average_samples", 3),
            color_interval=r.get("color_interval", 0.2),
            magnetic_interval=r.get("magnetic_interval", 0.2),
        )
    
    # Parse test settings
    if "test" in data:
        config.duration = data["test"].get("duration", 0)
        config.verbose = data["test"].get("verbose", True)
    
    return config


# =============================================================================
# TEST RUNNER
# =============================================================================

class TestRunner:
    """Unified test runner for MACRO."""
    
    def __init__(self, config: TestConfig):
        self.config = config
        self.sensors: Optional[SensorInput] = None
        self.motion: Optional[MotionController] = None
        self.navigator: Optional[Navigation] = None
        self.display: Optional[NavigationDisplay] = None
        self.payload_motor: Optional[Motor] = None
        self.central_pos = 0
        self._running = False
        self._log_file = None
        self._start_time = 0.0
        self._last_console_time = 0.0
    
    def initialize(self):
        """Initialize all enabled systems."""
        cfg = self.config
        
        if cfg.verbose:
            print("=== MACRO Test Runner ===")
            print(f"Features: nav={cfg.features.navigation}, mobility={cfg.features.mobility}, "
                  f"display={cfg.features.display}, safety={cfg.features.safety_ring}")
            print(f"Mode: {'manual' if cfg.features.manual_control else 'autonomous'}")
        
        # Initialize sensors
        self.sensors = SensorInput(
            imu=cfg.sensors.imu,
            ultrasonic=cfg.sensors.ultrasonic,
            ultrasonic_pin=cfg.sensors.ultrasonic_pin,
            line_finders=cfg.sensors.line_finders,
            line_finder_left_pin=cfg.sensors.line_finder_left_pin,
            line_finder_right_pin=cfg.sensors.line_finder_right_pin,
            button=cfg.sensors.button,
            button_pin=cfg.sensors.button_pin,
            color_sensor=cfg.sensors.color_sensor,
            color_sensor_port=cfg.sensors.color_sensor_port,
        )
        
        # Initialize mobility
        if cfg.features.mobility:
            self.motion = MotionController(
                front_motor=cfg.mobility.front_motor,
                turn_motor=cfg.mobility.turn_motor,
                sensors=self.sensors,
                slowdown_distance=cfg.mobility.slowdown_distance,
                stopping_distance=cfg.mobility.stopping_distance,
                forward_speed=cfg.mobility.forward_speed,
                forward_speed_slow=cfg.mobility.forward_speed_slow,
            )
        
        # Initialize navigation
        if cfg.features.navigation:
            self.navigator = Navigation(
                sensors=self.sensors,
                position=cfg.navigation.position,
                orientation=cfg.navigation.orientation,
                mode="degrees",
                velocity_decay=cfg.navigation.velocity_decay,
                accel_threshold=cfg.navigation.accel_threshold,
                motor_velocity_threshold=cfg.navigation.motor_velocity_threshold,
                motion_controller=self.motion,
            )
        
        # Initialize display
        if cfg.features.display and self.navigator:
            self.display = NavigationDisplay(
                width=cfg.display.width,
                height=cfg.display.height,
                scale=cfg.display.scale,
                navigator=self.navigator,
            )
            self.display.set_world_bounds(cfg.display.world_min, cfg.display.world_max)
        
        # Initialize payload motor
        if cfg.features.payload:
            self.payload_motor = Motor(cfg.payload.motor)
        
        if cfg.verbose:
            print("Systems initialized.")
        
        # Initialize logging
        self._init_logging()
    
    def _init_logging(self):
        """Initialize file logging if enabled."""
        cfg = self.config.logging
        if not cfg.enabled:
            return
        
        import time
        from datetime import datetime
        
        # Create logs directory if needed
        log_path = Path(cfg.file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        
        self._start_time = time.time()
        
        # Open log file
        self._log_file = open(log_path, 'w')
        
        if cfg.format == "csv":
            # Write CSV header
            headers = ["timestamp"]
            if cfg.log_position:
                headers.extend(["pos_x", "pos_y", "pos_z"])
            if cfg.log_velocity:
                headers.extend(["vel_x", "vel_y", "vel_z"])
            if cfg.log_orientation:
                headers.extend(["yaw", "pitch", "roll"])
            if cfg.log_acceleration:
                headers.extend(["accel_x", "accel_y", "accel_z"])
            if cfg.log_magnetic_field:
                headers.append("magnetic_magnitude")
            if cfg.log_motor_velocity:
                headers.append("motor_velocity")
            if cfg.log_distance:
                headers.append("distance")
            self._log_file.write(",".join(headers) + "\n")
        elif cfg.format == "json":
            self._log_file.write("[\n")
        
        if self.config.verbose:
            print(f"Logging to {cfg.file} ({cfg.format} format)")
    
    def _log_state(self):
        """Log current state to file."""
        if not self._log_file or not self.navigator:
            return
        
        import time
        from datetime import datetime
        
        cfg = self.config.logging
        
        # Get timestamp
        if cfg.timestamp_format == "elapsed":
            timestamp = time.time() - self._start_time
            ts_str = f"{timestamp:.3f}"
        elif cfg.timestamp_format == "absolute":
            timestamp = time.time()
            ts_str = f"{timestamp:.3f}"
        else:  # iso
            ts_str = datetime.now().isoformat()
        
        if cfg.format == "csv":
            values = [ts_str]
            if cfg.log_position:
                x, y, z = self.navigator.get_position()
                values.extend([f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])
            if cfg.log_velocity:
                vx, vy, vz = self.navigator.velocity
                values.extend([f"{vx:.6f}", f"{vy:.6f}", f"{vz:.6f}"])
            if cfg.log_orientation:
                yaw, pitch, roll = self.navigator.orientation
                values.extend([f"{yaw:.3f}", f"{pitch:.3f}", f"{roll:.3f}"])
            if cfg.log_acceleration:
                ax, ay, az = self.navigator.acceleration
                values.extend([f"{ax:.6f}", f"{ay:.6f}", f"{az:.6f}"])
            if cfg.log_magnetic_field:
                values.append(f"{self.navigator.magnetic_magnitude:.2f}")
            if cfg.log_motor_velocity and self.motion:
                values.append(f"{self.motion.motor_velocity:.3f}")
            if cfg.log_distance and self.sensors:
                # Note: This would need async handling in real usage
                values.append("0.0")
            self._log_file.write(",".join(values) + "\n")
        
        elif cfg.format == "json":
            import json
            entry = {"timestamp": ts_str}
            if cfg.log_position:
                x, y, z = self.navigator.get_position()
                entry["position"] = {"x": x, "y": y, "z": z}
            if cfg.log_velocity:
                vx, vy, vz = self.navigator.velocity
                entry["velocity"] = {"x": vx, "y": vy, "z": vz}
            if cfg.log_orientation:
                yaw, pitch, roll = self.navigator.orientation
                entry["orientation"] = {"yaw": yaw, "pitch": pitch, "roll": roll}
            if cfg.log_acceleration:
                ax, ay, az = self.navigator.acceleration
                entry["acceleration"] = {"x": ax, "y": ay, "z": az}
            if cfg.log_magnetic_field:
                entry["magnetic_magnitude"] = self.navigator.magnetic_magnitude
            if cfg.log_motor_velocity and self.motion:
                entry["motor_velocity"] = self.motion.motor_velocity
            self._log_file.write(json.dumps(entry) + ",\n")
        
        elif cfg.format == "text":
            line = f"[{ts_str}]"
            if cfg.log_position:
                x, y, z = self.navigator.get_position()
                line += f" pos=({x:.3f},{y:.3f},{z:.3f})"
            if cfg.log_velocity:
                vx, vy, vz = self.navigator.velocity
                line += f" vel=({vx:.3f},{vy:.3f},{vz:.3f})"
            if cfg.log_orientation:
                yaw, pitch, roll = self.navigator.orientation
                line += f" ori=({yaw:.1f},{pitch:.1f},{roll:.1f})"
            self._log_file.write(line + "\n")
    
    async def run(self):
        """Run the test based on configuration."""
        self._running = True
        cfg = self.config
        
        try:
            # Calibrate navigation if enabled
            if cfg.features.navigation and cfg.navigation.calibrate:
                if cfg.verbose:
                    print("Calibrating IMU... Keep robot stationary.")
                await self.navigator.calibrate(
                    samples=cfg.navigation.calibration_samples,
                    delay=cfg.navigation.calibration_delay,
                )
                if cfg.verbose:
                    print("Calibration complete.")
            
            # Build task list
            tasks = []
            
            # Navigation update task
            if cfg.features.navigation:
                tasks.append(self._run_navigation())
            
            # Motor tracking task
            if cfg.features.mobility:
                tasks.append(self._run_motor_tracking())
            
            # Safety ring task
            if cfg.features.mobility and cfg.features.safety_ring:
                tasks.append(self.motion.start_safety_ring())
            
            # Display task
            if cfg.features.display and self.display:
                tasks.append(self._run_display())
            
            # Magnetism monitoring task
            if cfg.features.magnetism and self.navigator:
                tasks.append(self._run_magnetism())
            
            # Control mode
            if cfg.features.manual_control:
                tasks.append(self._run_manual_control())
            elif cfg.features.autonomous and cfg.features.mobility:
                # Start motors in autonomous mode
                self.motion.start()
                if cfg.verbose:
                    print("Started autonomous movement.")
            
            # Run all tasks
            if tasks:
                await asyncio.gather(*tasks)
            else:
                print("No tasks to run. Check feature configuration.")
        
        except KeyboardInterrupt:
            if cfg.verbose:
                print("\nTest interrupted.")
        finally:
            self.stop()
    
    async def _run_navigation(self):
        """Run continuous navigation updates."""
        cfg = self.config.navigation
        console_cfg = self.config.console
        import time
        
        while self._running:
            await self.navigator.update_state(dt=cfg.update_interval)
            
            # Log to file
            if self.config.logging.enabled:
                self._log_state()
            
            # Also use navigator's internal logging if enabled
            if cfg.log_state:
                self.navigator.log_state(asyncio.get_event_loop().time())
            
            # Console output with separate interval
            if console_cfg.enabled:
                now = time.time()
                if now - self._last_console_time >= console_cfg.interval:
                    self._print_console_state()
                    self._last_console_time = now
            
            await asyncio.sleep(cfg.update_interval)
    
    async def _run_motor_tracking(self):
        """Update motor encoder state continuously."""
        cfg = self.config.navigation
        while self._running:
            await self.motion.update_motor_state(dt=cfg.update_interval)
            await asyncio.sleep(cfg.update_interval)
    
    async def _run_display(self):
        """Run the navigation display."""
        cfg = self.config.navigation
        await self.display.run_continuous(update_interval=cfg.update_interval)
    
    async def _run_magnetism(self):
        """Monitor magnetic field magnitude."""
        while self._running:
            mag = await self.navigator.get_magnetic_field()
            print(f"Magnetic field: {mag:.2f} µT")
            await asyncio.sleep(0.2)
    
    async def _run_manual_control(self):
        """Run manual keyboard control."""
        cfg = self.config
        
        print("\n=== Manual Control ===")
        print("w=forward, s=stop, r=reverse, a=left, d=right, q=straighten")
        if cfg.features.payload:
            print("p=deploy payload, o=retract, l=stop payload")
        print("+=faster, -=slower, x=exit")
        print("=======================\n")
        
        import termios
        import tty
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setcbreak(fd)
            
            while self._running:
                # Non-blocking read with timeout
                import select
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    await self._handle_key(key)
                    if key == 'x':
                        break
                
                await asyncio.sleep(0.05)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    async def _handle_key(self, key: str):
        """Handle a single keypress."""
        cfg = self.config
        
        if key == 'w':
            self.motion.start()
            print("Forward")
        elif key == 's':
            self.motion.stop()
            print("Stop")
        elif key == 'r':
            self.motion.front_motor.start(cfg.mobility.reverse_speed)
            print("Reverse")
        elif key == 'a':
            new_pos = self.central_pos - cfg.manual_control.turn_amount
            if new_pos >= -cfg.manual_control.max_turn:
                self.motion.turn_left(cfg.manual_control.turn_amount)
                self.central_pos = new_pos
                print(f"Turn left (pos: {self.central_pos})")
        elif key == 'd':
            new_pos = self.central_pos + cfg.manual_control.turn_amount
            if new_pos <= cfg.manual_control.max_turn:
                self.motion.turn_right(cfg.manual_control.turn_amount)
                self.central_pos = new_pos
                print(f"Turn right (pos: {self.central_pos})")
        elif key == 'q':
            self.motion.straighten(self.central_pos)
            self.central_pos = 0
            print("Straightened")
        elif key == '+':
            cfg.mobility.forward_speed = min(cfg.mobility.forward_speed + 5, 100)
            self.motion.forward_speed = cfg.mobility.forward_speed
            print(f"Speed: {cfg.mobility.forward_speed}")
        elif key == '-':
            cfg.mobility.forward_speed = max(cfg.mobility.forward_speed - 5, 5)
            self.motion.forward_speed = cfg.mobility.forward_speed
            print(f"Speed: {cfg.mobility.forward_speed}")
        elif key == 'p' and cfg.features.payload and self.payload_motor:
            self.payload_motor.start(cfg.payload.speed)
            print("Payload deploying")
        elif key == 'o' and cfg.features.payload and self.payload_motor:
            self.payload_motor.start(-cfg.payload.speed)
            print("Payload retracting")
        elif key == 'l' and cfg.features.payload and self.payload_motor:
            self.payload_motor.stop()
            print("Payload stopped")
    
    def _print_console_state(self):
        """Print current state to console using console config."""
        if not self.navigator:
            return
        
        cfg = self.config.console
        prec = cfg.precision
        
        if cfg.format == "detailed":
            if cfg.print_position:
                x, y, z = self.navigator.get_position()
                print(f"Position:    ({x:.{prec}f}, {y:.{prec}f}, {z:.{prec}f}) m")
            if cfg.print_velocity:
                vx, vy, vz = self.navigator.velocity
                print(f"Velocity:    ({vx:.{prec}f}, {vy:.{prec}f}, {vz:.{prec}f}) m/s")
            if cfg.print_orientation:
                yaw, pitch, roll = self.navigator.orientation
                print(f"Orientation: ({yaw:.1f}, {pitch:.1f}, {roll:.1f}) deg")
            if cfg.print_acceleration:
                ax, ay, az = self.navigator.acceleration
                print(f"Accel:       ({ax:.{prec}f}, {ay:.{prec}f}, {az:.{prec}f}) m/s²")
            if cfg.print_magnetic_field:
                print(f"Magnetic:    {self.navigator.magnetic_magnitude:.2f} µT")
            if cfg.print_motor_velocity and self.motion:
                print(f"Motor Vel:   {self.motion.motor_velocity:.2f} deg/s")
            print()  # Blank line between updates
        else:  # compact
            parts = []
            if cfg.print_position:
                x, y, z = self.navigator.get_position()
                parts.append(f"pos=({x:.{prec}f},{y:.{prec}f},{z:.{prec}f})")
            if cfg.print_velocity:
                vx, vy, vz = self.navigator.velocity
                parts.append(f"vel=({vx:.{prec}f},{vy:.{prec}f},{vz:.{prec}f})")
            if cfg.print_orientation:
                yaw, pitch, roll = self.navigator.orientation
                parts.append(f"ori=({yaw:.1f},{pitch:.1f},{roll:.1f})")
            if cfg.print_acceleration:
                ax, ay, az = self.navigator.acceleration
                parts.append(f"acc=({ax:.{prec}f},{ay:.{prec}f},{az:.{prec}f})")
            if cfg.print_magnetic_field:
                parts.append(f"mag={self.navigator.magnetic_magnitude:.1f}")
            if cfg.print_motor_velocity and self.motion:
                parts.append(f"mot={self.motion.motor_velocity:.1f}")
            if cfg.print_distance:
                parts.append("dist=?")  # Would need async
            print(" | ".join(parts))
    
    def _print_state(self):
        """Legacy print state method for backwards compatibility."""
        self._print_console_state()
    
    def stop(self):
        """Stop all systems."""
        self._running = False
        
        if self.motion:
            self.motion.stop()
        
        if self.payload_motor:
            self.payload_motor.stop()
        
        # Close log file
        if self._log_file:
            if self.config.logging.format == "json":
                # Remove trailing comma and close JSON array
                self._log_file.seek(self._log_file.tell() - 2)
                self._log_file.write("\n]\n")
            self._log_file.close()
            if self.config.verbose:
                print(f"Log saved to {self.config.logging.file}")
        
        if self.config.verbose:
            print("Test stopped.")
            if self.navigator and self.navigator.log:
                print(f"Logged {len(self.navigator.log)} entries over "
                      f"{self.navigator.log[-1]['timestamp']:.2f} seconds.")


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Main entry point."""
    # Get config path from command line or use default
    config_path = sys.argv[1] if len(sys.argv) > 1 else None
    
    # Load configuration
    config = load_test_config(config_path)
    
    # Create and run test
    runner = TestRunner(config)
    runner.initialize()
    
    try:
        asyncio.run(runner.run())
    except KeyboardInterrupt:
        runner.stop()


if __name__ == "__main__":
    main()
