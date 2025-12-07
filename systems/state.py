from dataclasses import dataclass, field
import numpy as np

@dataclass
class State:
    """Class for tracking the robot's state."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # x, y, z position in meters
    sensor_positions: dict = None  # Positions of various sensors
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # x, y, z velocity in m/s
    acceleration_raw: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Raw acceleration from IMU in m/s²
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))  # x, y, z acceleration in m/s²
    orientation: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Orientation as Euler angles (yaw, pitch, roll)
    angular_velocity_raw: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Raw angular velocity from IMU in deg/s
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Bias-corrected angular velocity in deg/s
    angular_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Angular acceleration in deg/s²
    magnetic_field: float = 0.0  # Magnetic field magnitude in µT
    calibrated_position: bool = False  # Whether position/acceleration is calibrated
    calibrated_orientation: bool = False  # Whether orientation/gyro is calibrated
    calibrated_mag: bool = False  # Whether magnetic field is calibrated
    bias: dict = None  # Calibration bias values
    lf_left_value: float = 0.0  # Left line finder sensor value
    lf_right_value: float = 0.0  # Right line finder sensor value
    ultrasonic_distance: float = 30.0  # Ultrasonic sensor distance in cm
    color_sensor_value: int = 0  # Color sensor value 
    button_pressed: bool = False  # Button state
    motor_position: float = 0.0  # Motor position in degrees
    motor_velocity: float = 0.0  # Motor velocity in degrees per second