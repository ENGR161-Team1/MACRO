"""                                                                                                                                                
navigation_system.py

3D Navigation and Position Tracking System for MACRO.

This module provides:
- Transformation3D: 3D rotation and translation utilities using Euler angles
- Location3D: Position tracking using IMU sensor data with dead reckoning
- Navigation3D: Extended position tracking with timestamped logging

Classes:
    Transformation3D: Handles 3D coordinate transformations (rotation/translation)
    Location3D: Tracks position, velocity, and orientation using IMU integration
    Navigation3D: Extends Location3D with continuous update loop and logging
"""

import asyncio
import numpy as np
import math
from basehat import IMUSensor

# Gravity constant (m/s^2)
GRAVITY = 9.80235


class Transformation3D:
    """
    3D transformation utilities for rotation and translation operations.
    
    Supports Euler angle rotations (yaw, pitch, roll) and vector operations.
    All rotation matrices follow the right-hand rule convention.
    
    Args:
        mode (str): Angle unit mode - "degrees" (default) or "radians"
    """
    
    def __init__(self, **kwargs):
        self.mode = kwargs.get("mode", "degrees")

    async def get_rotation_yaw(self, **kwargs):
        """Generate rotation matrix for yaw (Z-axis rotation)."""
        yaw = kwargs.get("yaw", 0.0)
        invert = kwargs.get("invert", False)
        
        if self.mode == "degrees":
            yaw = math.radians(yaw)

        R_yaw = np.array([
            [ math.cos(yaw),  math.sin(yaw), 0],
            [-math.sin(yaw),  math.cos(yaw), 0],
            [0,               0,             1]
        ])
        return np.transpose(R_yaw) if invert else R_yaw
    
    async def get_rotation_pitch(self, **kwargs):
        """Generate rotation matrix for pitch (Y-axis rotation)."""
        pitch = kwargs.get("pitch", 0.0)
        invert = kwargs.get("invert", False)
        
        if self.mode == "degrees":
            pitch = math.radians(pitch)

        R_pitch = np.array([
            [ math.cos(pitch), 0, -math.sin(pitch)],
            [0,                1,  0              ],
            [ math.sin(pitch), 0,  math.cos(pitch)]
        ])
        return np.transpose(R_pitch) if invert else R_pitch
    
    async def get_rotation_roll(self, **kwargs):
        """Generate rotation matrix for roll (X-axis rotation)."""
        roll = kwargs.get("roll", 0.0)
        invert = kwargs.get("invert", False)
        
        if self.mode == "degrees":
            roll = math.radians(roll)

        R_roll = np.array([
            [1, 0,               0              ],
            [0, math.cos(roll),  math.sin(roll) ],
            [0, -math.sin(roll), math.cos(roll) ]
        ])
        return np.transpose(R_roll) if invert else R_roll
    
    async def get_rotation(self, **kwargs):
        """
        Generate combined rotation matrix from yaw, pitch, and roll.
        
        Rotation order: Yaw -> Pitch -> Roll (ZYX convention)
        """
        yaw = kwargs.get("yaw", 0.0)
        pitch = kwargs.get("pitch", 0.0)
        roll = kwargs.get("roll", 0.0)
        invert = kwargs.get("invert", False)
        
        R_yaw = await self.get_rotation_yaw(yaw=yaw, invert=invert)
        R_pitch = await self.get_rotation_pitch(pitch=pitch, invert=invert)
        R_roll = await self.get_rotation_roll(roll=roll, invert=invert)
        
        return np.matmul(R_yaw, np.matmul(R_pitch, R_roll))
    
    async def rotate_vector(self, **kwargs):
        """Apply rotation to a 3D vector."""
        vector = np.array(kwargs.get("vector", [0.0, 0.0, 0.0]))
        yaw = kwargs.get("yaw", 0.0)
        pitch = kwargs.get("pitch", 0.0)
        roll = kwargs.get("roll", 0.0)
        invert = kwargs.get("invert", False)
        
        R = await self.get_rotation(yaw=yaw, pitch=pitch, roll=roll, invert=invert)
        return np.matmul(R, vector)
    
    async def translate_vector(self, **kwargs):
        """Apply translation to a 3D vector."""
        vector = np.array(kwargs.get("vector", [0.0, 0.0, 0.0]))
        translation = np.array(kwargs.get("translation", [0.0, 0.0, 0.0]))
        return vector + translation


class Location3D:
    """
    3D position tracking using IMU sensor data.
    
    Uses dead reckoning to estimate position by integrating acceleration
    and gyroscope data. Position is tracked in a global reference frame.
    
    Args:
        position (list): Initial position [x, y, z] in meters
        orientation (list): Initial orientation [yaw, pitch, roll] in degrees
        imu (IMUSensor): IMU sensor instance for reading acceleration/gyro data
        mode (str): Angle unit mode - "degrees" (default) or "radians"
    
    Attributes:
        pos: Current position [x, y, z] in global frame
        velocity: Current velocity [vx, vy, vz] in global frame
        acceleration: Current acceleration [ax, ay, az] in global frame (gravity-compensated)
        orientation: Current orientation [yaw, pitch, roll]
    """
    
    def __init__(self, **kwargs):
        # Position state (global frame)
        self.pos = np.array(kwargs.get("position", [0.0, 0.0, 0.0]))
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        self.accel_local = np.array([0.0, 0.0, 0.0])
        
        # Orientation state [yaw, pitch, roll]
        self.orientation = np.array(kwargs.get("orientation", [0.0, 0.0, 0.0]))
        self.d_orientation = np.array([0.0, 0.0, 0.0])  # Angular velocity
        self.a_orientation = np.array([0.0, 0.0, 0.0])  # Angular acceleration
        
        # Calibration offsets (measured when stationary)
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.mag_baseline = 0.0
        self.calibrated = False
        
        # Velocity decay factor to reduce drift (0.0 = no decay, 1.0 = instant stop)
        self.velocity_decay = kwargs.get("velocity_decay", 0.02)
        
        # Threshold for considering acceleration as noise (m/s^2)
        self.accel_threshold = kwargs.get("accel_threshold", 0.1)
        
        # Components
        self.transformer = Transformation3D(**kwargs)
        self.imu = kwargs.get("imu", None)
        self.initialized = False
    
    async def calibrate(self, **kwargs):
        """
        Calibrate the IMU by measuring bias while stationary.
        
        Call this method when the robot is completely still.
        Takes multiple samples and averages them.
        
        Args:
            samples (int): Number of samples to average (default: 50)
            delay (float): Delay between samples in seconds (default: 0.02)
        """
        samples = kwargs.get("samples", 50)
        delay = kwargs.get("delay", 0.02)
        
        if self.imu is None:
            print("No IMU sensor provided.")
            return False
        
        print("Calibrating IMU... Keep robot stationary.")
        
        accel_sum = np.array([0.0, 0.0, 0.0])
        gyro_sum = np.array([0.0, 0.0, 0.0])
        mag_sum = 0.0
        
        for i in range(samples):
            ax, ay, az = self.imu.getAccel()
            gx, gy, gz = self.imu.getGyro()
            
            accel_sum += np.array([ax, ay, az])
            gyro_sum += np.array([gz, gy, gx])  # yaw, pitch, roll order
            mag_sum += self.get_magnetic_field()
            
            await asyncio.sleep(delay)
        
        # Average the readings
        self.accel_bias = accel_sum / samples
        self.gyro_bias = gyro_sum / samples
        self.mag_baseline = mag_sum / samples
        
        # The Z acceleration bias should preserve gravity
        # We want to measure what "zero" acceleration looks like in the local frame
        # When stationary and level, accel should read (0, 0, ~9.81)
        # So we store the full reading and subtract it later, then add back gravity in global frame
        
        self.calibrated = True
        print(f"Calibration complete.")
        print(f"  Accel bias: {self.accel_bias}")
        print(f"  Gyro bias: {self.gyro_bias}")
        print(f"  Mag baseline: {self.mag_baseline:.2f} µT")
        
        return True
    
    async def update_orientation(self, **kwargs):
        """
        Update orientation by integrating gyroscope data.
        
        Args:
            dt (float): Time step in seconds
        """
        dt = kwargs.get("dt", 0.1)
        
        if self.imu is None:
            print("No IMU sensor provided.")
            return False
        
        # Get angular velocity from gyroscope (degrees/second)
        d_roll, d_pitch, d_yaw = self.imu.getGyro()
        new_d_orientation = np.array([d_yaw, d_pitch, d_roll])
        
        # Subtract gyro bias if calibrated
        if self.calibrated:
            new_d_orientation -= self.gyro_bias
        
        if not self.initialized:
            # First iteration: initialize rates without calculating acceleration
            self.d_orientation = new_d_orientation
            self.initialized = True
        else:
            # Integrate orientation using trapezoidal approximation
            self.orientation += 0.5 * self.a_orientation * (dt ** 2) + self.d_orientation * dt
            self.a_orientation = (new_d_orientation - self.d_orientation) / dt
            self.d_orientation = new_d_orientation
        
        return True
    
    async def update_position(self, **kwargs):
        """
        Update position by integrating accelerometer data.
        
        Transforms local acceleration to global frame and subtracts gravity.
        Applies calibration bias and noise thresholding.
        
        Uses proper integration order:
        1. Update position using previous velocity and acceleration
        2. Update velocity using previous acceleration
        3. Calculate new acceleration for next iteration
        
        Args:
            dt (float): Time step in seconds
            display (bool): Print position data if True
        """
        dt = kwargs.get("dt", 0.1)
        display = kwargs.get("display", False)
        
        if self.imu is None:
            print("No IMU sensor provided.")
            return False
        
        # Store previous acceleration and velocity for integration
        prev_acceleration = self.acceleration.copy()
        prev_velocity = self.velocity.copy()
        
        # Update position FIRST using previous velocity and acceleration
        # p = p0 + v0*dt + 0.5*a0*dt^2
        self.pos = await self.transformer.translate_vector(
            vector=self.pos,
            translation=prev_velocity * dt + 0.5 * prev_acceleration * (dt ** 2)
        )
        
        # Update velocity SECOND using previous acceleration
        # v = v0 + a0*dt
        self.velocity = await self.transformer.translate_vector(
            vector=prev_velocity,
            translation=prev_acceleration * dt
        )
        
        # Apply velocity decay to reduce drift when acceleration is near zero
        if np.linalg.norm(prev_acceleration) < self.accel_threshold:
            self.velocity *= (1.0 - self.velocity_decay)
        
        # NOW read new acceleration for next iteration
        ax, ay, az = self.imu.getAccel()
        self.accel_local = np.array([ax, ay, az])
        
        # Subtract calibration bias if calibrated
        if self.calibrated:
            self.accel_local -= self.accel_bias
        
        # Update orientation from gyroscope
        await self.update_orientation(dt=dt)
        
        # Transform local acceleration to global frame
        accel_global = await self.transformer.rotate_vector(
            vector=self.accel_local,
            yaw=self.orientation[0],
            pitch=self.orientation[1],
            roll=self.orientation[2],
            invert=True
        )
        
        # If not calibrated, remove gravity (calibrated bias already includes gravity)
        if not self.calibrated:
            accel_global -= np.array([0.0, 0.0, GRAVITY])
        
        # Apply noise threshold - treat small accelerations as zero
        for i in range(3):
            if abs(accel_global[i]) < self.accel_threshold:
                accel_global[i] = 0.0
        
        # Store new acceleration for next iteration
        self.acceleration = accel_global
        
        if display:
            print(f"Position: {self.pos}, Velocity: {self.velocity}, Acceleration: {self.acceleration}")
        
        return True
    
    def get_position(self):
        """
        Get the current position as a tuple.
        
        Returns:
            tuple: Current position (x, y, z) in meters
        """
        return tuple(self.pos)


class Navigation3D(Location3D):
    """
    3D navigation system with logging capabilities.
    
    Extends Location3D to add timestamped logging of position, velocity,
    orientation, and acceleration data.
    
    Args:
        Inherits all arguments from Location3D
    
    Attributes:
        log: List of logged navigation data entries
        start_time: Timestamp when navigation started
    """
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.log = []
        self.start_time = None
        self.magnetic_field = np.array([0.0, 0.0, 0.0])
        self.magnetic_magnitude = 0.0
    
    def get_magnetic_field(self):
        """
        Read and return the magnetic field magnitude.
        
        Returns:
            float: Magnitude of magnetic field in micro-tesla
        """
        if self.imu is None:
            return 0.0
        
        x_mag, y_mag, z_mag = self.imu.getMag()
        self.magnetic_field = np.array([x_mag, y_mag, z_mag])
        self.magnetic_magnitude = np.linalg.norm(self.magnetic_field)
        
        return self.magnetic_magnitude
    
    def log_state(self, timestamp):
        """
        Log the current navigation state with timestamp.
        
        Args:
            timestamp (float): Current timestamp in seconds since start
        """
        entry = {
            "timestamp": timestamp,
            "position": self.pos.copy(),
            "velocity": self.velocity.copy(),
            "acceleration": self.acceleration.copy(),
            "orientation": self.orientation.copy()
        }
        self.log.append(entry)
    
    def print_state(self, timestamp):
        """
        Print the current navigation state with timestamp.
        
        Args:
            timestamp (float): Current timestamp in seconds since start
        """
        position = tuple(self.pos)
        velocity = tuple(self.velocity)
        acceleration = tuple(self.acceleration)
        orientation = tuple(self.orientation)
        print(f"[{timestamp:.3f}s] Pos: {position}, Vel: {velocity}, "
              f"Acc: {acceleration}, Orient: {orientation}")
    
    async def run_continuous_update(self, **kwargs):
        """
        Continuously update position at a fixed interval with optional logging and printing.
        
        Args:
            update_interval (float): Update interval in seconds (default: 0.1)
            log_state (bool): Whether to log state each iteration (default: True)
            print_state (bool): Whether to print state each iteration (default: False)
            calibrate (bool): Whether to calibrate IMU before starting (default: True)
            calibration_samples (int): Number of calibration samples (default: 50)
        """
        import time
        
        update_interval = kwargs.get("update_interval", 0.1)
        log_state_enabled = kwargs.get("log_state", True)
        print_state_enabled = kwargs.get("print_state", False)
        do_calibrate = kwargs.get("calibrate", True)
        calibration_samples = kwargs.get("calibration_samples", 50)
        
        # Calibrate IMU if requested
        if do_calibrate and not self.calibrated:
            await self.calibrate(samples=calibration_samples)
        
        self.start_time = time.time()
        
        while True:
            await self.update_position(dt=update_interval)
            
            # Get current timestamp
            timestamp = time.time() - self.start_time
            
            # Log current state if enabled
            if log_state_enabled:
                self.log_state(timestamp)
            
            # Print current state if enabled
            if print_state_enabled:
                self.print_state(timestamp)
            
            await asyncio.sleep(update_interval)


if __name__ == "__main__":
    # Example usage
    imu_sensor = IMUSensor()
    navigator = Navigation3D(imu=imu_sensor, mode="degrees")
    
    async def main():
        await navigator.run_continuous_update(
            update_interval=0.1,
            log_state=True,
            print_state=True,
            calibrate=True
        )
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print(f"Stopping navigation. Logged {len(navigator.log)} entries across {navigator.log[-1]['timestamp']:.2f} seconds.")