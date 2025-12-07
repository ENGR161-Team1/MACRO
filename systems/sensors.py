"""
sensors.py

Centralized sensor management for MACRO.

This module provides:
- SensorInput: Unified interface for all hardware sensors

The SensorInput class manages all sensors (IMU, ultrasonic, line finders, etc.)
and provides a single point of access for navigation and mobility systems.
"""

import numpy as np
from basehat import IMUSensor, UltrasonicSensor, Button, HallSensor


class SensorInput:
    """
    Centralized sensor management class.
    
    Provides unified access to all hardware sensors for navigation,
    mobility, and other systems.
    
    Args:
        imu (bool): Enable IMU sensor (default: True)
        ultrasonic_pin (int): GPIO pin for ultrasonic sensor (default: 26)
        ultrasonic (bool): Enable ultrasonic sensor (default: True)
        line_finder_left_pin (int): GPIO pin for left line finder (default: 16)
        line_finder_right_pin (int): GPIO pin for right line finder (default: 5)
        line_finders (bool): Enable line finder sensors (default: False)
        button_pin (int): GPIO pin for button (default: 22)
        button (bool): Enable button (default: False)
        hall_pin (int): GPIO pin for hall sensor (default: 12)
        hall (bool): Enable hall sensor (default: False)
    
    Attributes:
        imu: IMUSensor instance or None
        ultrasonic: UltrasonicSensor instance or None
        line_finder_left: LineFinder instance or None
        line_finder_right: LineFinder instance or None
        button: Button instance or None
        hall: HallSensor instance or None
    """
    
    def __init__(self, **kwargs):
        # IMU sensor
        if kwargs.get("imu", True):
            self.imu = IMUSensor()
        else:
            self.imu = None
        
        # Ultrasonic sensor
        if kwargs.get("ultrasonic", True):
            ultrasonic_pin = kwargs.get("ultrasonic_pin", 26)
            self.ultrasonic = UltrasonicSensor(ultrasonic_pin)
        else:
            self.ultrasonic = None
        
        # Line finder sensors (disabled by default)
        if kwargs.get("line_finders", False):
            # Note: LineFinder class not yet implemented
            # left_pin = kwargs.get("line_finder_left_pin", 16)
            # right_pin = kwargs.get("line_finder_right_pin", 5)
            # self.line_finder_left = LineFinder(left_pin)
            # self.line_finder_right = LineFinder(right_pin)
            self.line_finder_left = None
            self.line_finder_right = None
        else:
            self.line_finder_left = None
            self.line_finder_right = None
        
        # Button (disabled by default)
        if kwargs.get("button", False):
            button_pin = kwargs.get("button_pin", 22)
            self.button_sensor = Button(button_pin)
        else:
            self.button_sensor = None
        
        # Hall sensor (disabled by default)
        if kwargs.get("hall", False):
            hall_pin = kwargs.get("hall_pin", 12)
            self.hall = HallSensor(hall_pin)
        else:
            self.hall = None
        
        # Cached sensor values
        self._accel = np.array([0.0, 0.0, 0.0])
        self._gyro = np.array([0.0, 0.0, 0.0])
        self._mag = np.array([0.0, 0.0, 0.0])
        self._distance = 0.0
    
    # -------------------------------------------------------------------------
    # IMU Methods
    # -------------------------------------------------------------------------
    
    def get_accel(self):
        """
        Get acceleration from IMU.
        
        Returns:
            tuple: (ax, ay, az) in m/s²
        """
        if self.imu is None:
            return (0.0, 0.0, 0.0)
        
        self._accel = np.array(self.imu.getAccel())
        return tuple(self._accel)
    
    def get_gyro(self):
        """
        Get angular velocity from IMU.
        
        Returns:
            tuple: (gx, gy, gz) in degrees/second
        """
        if self.imu is None:
            return (0.0, 0.0, 0.0)
        
        self._gyro = np.array(self.imu.getGyro())
        return tuple(self._gyro)
    
    def get_mag(self):
        """
        Get magnetic field from IMU.
        
        Returns:
            tuple: (mx, my, mz) in micro-tesla
        """
        if self.imu is None:
            return (0.0, 0.0, 0.0)
        
        self._mag = np.array(self.imu.getMag())
        return tuple(self._mag)
    
    def get_magnetic_magnitude(self):
        """
        Get magnetic field magnitude.
        
        Returns:
            float: Magnitude in micro-tesla
        """
        if self.imu is None:
            return 0.0
        
        self._mag = np.array(self.imu.getMag())
        return float(np.linalg.norm(self._mag))
    
    # -------------------------------------------------------------------------
    # Ultrasonic Methods
    # -------------------------------------------------------------------------
    
    def get_distance(self):
        """
        Get distance from ultrasonic sensor.
        
        Returns:
            float: Distance in centimeters
        """
        if self.ultrasonic is None:
            return -1.0
        
        try:
            self._distance = float(self.ultrasonic.getDist)
        except:
            self._distance = -1.0
        
        return self._distance
    
    # -------------------------------------------------------------------------
    # Button Methods
    # -------------------------------------------------------------------------
    
    def is_button_pressed(self):
        """
        Check if button is pressed.
        
        Returns:
            bool: True if pressed, False otherwise
        """
        if self.button_sensor is None:
            return False
        
        return self.button_sensor.is_pressed
    
    # -------------------------------------------------------------------------
    # Hall Sensor Methods
    # -------------------------------------------------------------------------
    
    def get_hall_value(self):
        """
        Get hall sensor reading.
        
        Returns:
            bool or int: Hall sensor value
        """
        if self.hall is None:
            return False
        
        return self.hall.read()
    
    # -------------------------------------------------------------------------
    # Line Finder Methods
    # -------------------------------------------------------------------------
    
    def get_line_left(self):
        """
        Get left line finder reading.
        
        Returns:
            bool: True if line detected
        """
        if self.line_finder_left is None:
            return False
        
        return self.line_finder_left.value
    
    def get_line_right(self):
        """
        Get right line finder reading.
        
        Returns:
            bool: True if line detected
        """
        if self.line_finder_right is None:
            return False
        
        return self.line_finder_right.value
    
    # -------------------------------------------------------------------------
    # Utility Methods
    # -------------------------------------------------------------------------
    
    def has_imu(self):
        """Check if IMU is available."""
        return self.imu is not None
    
    def has_ultrasonic(self):
        """Check if ultrasonic sensor is available."""
        return self.ultrasonic is not None
    
    def has_button(self):
        """Check if button is available."""
        return self.button_sensor is not None
    
    def has_hall(self):
        """Check if hall sensor is available."""
        return self.hall is not None
    
    def has_line_finders(self):
        """Check if line finders are available."""
        return self.line_finder_left is not None and self.line_finder_right is not None
