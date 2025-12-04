"""
navigation_system.py

3D Navigation and Position Tracking System for MACRO.

This module provides:
- Transformation3D: 3D rotation and translation utilities using Euler angles
- Location3D: Position tracking using IMU sensor data with dead reckoning

Classes:
    Transformation3D: Handles 3D coordinate transformations (rotation/translation)
    Location3D: Tracks position, velocity, and orientation using IMU integration
"""

import asyncio
import numpy as np
import math
from basehat import IMUSensor

# Gravity constant (m/s^2)
GRAVITY = 9.81


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
        
        # Components
        self.transformer = Transformation3D(**kwargs)
        self.imu = kwargs.get("imu", None)
        self.initialized = False
    
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
        
        Args:
            dt (float): Time step in seconds
            display (bool): Print position data if True
        """
        dt = kwargs.get("dt", 0.1)
        display = kwargs.get("display", False)
        
        if self.imu is None:
            print("No IMU sensor provided.")
            return False
        
        # Read local acceleration from IMU
        ax, ay, az = self.imu.getAccel()
        self.accel_local = np.array([ax, ay, az])
        
        # Update position: p = p0 + v*dt + 0.5*a*dt^2
        self.pos = await self.transformer.translate_vector(
            vector=self.pos,
            translation=self.velocity * dt + 0.5 * self.acceleration * (dt ** 2)
        )
        
        # Update velocity: v = v0 + a*dt
        self.velocity = await self.transformer.translate_vector(
            vector=self.velocity,
            translation=self.acceleration * dt
        )
        
        # Update orientation from gyroscope
        await self.update_orientation(dt=dt)
        
        # Transform local acceleration to global frame
        self.acceleration = await self.transformer.rotate_vector(
            vector=self.accel_local,
            yaw=self.orientation[0],
            pitch=self.orientation[1],
            roll=self.orientation[2],
            invert=True
        )
        
        # Remove gravity component (assuming Z-up global frame)
        self.acceleration -= np.array([0.0, 0.0, GRAVITY])
        
        if display:
            print(f"Position: {self.pos}, Velocity: {self.velocity}, Acceleration: {self.acceleration}")
        
        return True
    
    async def run_continuous_update(self, **kwargs):
        """
        Continuously update position at a fixed interval.
        
        Args:
            update_interval (float): Update interval in seconds (default: 0.1)
        """
        update_interval = kwargs.get("update_interval", 0.1)
        while True:
            await self.update_position(dt=update_interval)
            await asyncio.sleep(update_interval)


if __name__ == "__main__":
    # Example usage
    imu_sensor = IMUSensor()
    location_tracker = Location3D(imu=imu_sensor, mode="degrees")
    
    async def main():
        await location_tracker.run_continuous_update(update_interval=0.1)
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopping location tracking.")