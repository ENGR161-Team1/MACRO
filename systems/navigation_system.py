"""                                                                                                                                                
navigation_system.py

3D Navigation and Position Tracking System for MACRO.

This module provides:
- Transformation: 3D rotation and translation utilities using Euler angles
- Location: Position tracking using IMU sensor data with dead reckoning
- Navigation: Extended position tracking with timestamped logging

Classes:
    Transformation: Handles 3D coordinate transformations (rotation/translation)
    Location: Tracks position, velocity, and orientation using IMU integration
    Navigation: Extends Location with continuous update loop and logging
"""

import asyncio
import numpy as np
import math
from systems.state import State

# Gravity constant (m/s^2)
GRAVITY = 9.80235


class Transformation:
    """
    3D transformation utilities for rotation and translation operations.
    
    Supports Euler angle rotations (yaw, pitch, roll) and vector operations.
    All rotation matrices follow the right-hand rule convention.
    
    Args:
        mode (str): Angle unit mode - "degrees" (default) or "radians"
    """
    
    def __init__(self, **kwargs):
        self.mode = kwargs.get("mode", "degrees")

        # Remove uninitialized instance variables and use State for sensor heights
        self.state = State()
        self.state.imu_height = kwargs.get("imu_height", 0.015)  # Height of IMU from ground in meters
        self.state.color_sensor_height = kwargs.get("color_sensor_height", 0.025)  # Height of color sensor from ground in meters
        self.state.lf_height = kwargs.get("lf_height", 0.025)  # Height of left front distance sensor from ground in meters
        self.state.lf_offset = kwargs.get("lf_offset", 0.0225) # Horizontal offset of left front distance sensor from center in meters
        self.state.imu_to_lf = kwargs.get("imu_to_lf", 0.125) # Horizontal distance from IMU to front distance sensors in meters
        self.state.imu_to_color = kwargs.get("imu_to_color", 0.11) # Horizontal distance from IMU to color sensor in meters
        self.state.imu_to_cargo = kwargs.get("imu_to_cargo", 0.24) # Horizontal distance from IMU to cargo deploy location in meters

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


class Location:
    """
    3D position tracking using IMU sensor data.
    
    Uses dead reckoning to estimate position by integrating acceleration
    and gyroscope data. Position is tracked in a global reference frame.
    
    Args:
        position (list): Initial position [x, y, z] in meters
        orientation (list): Initial orientation [yaw, pitch, roll] in degrees
        sensors (SensorInput): Centralized sensor manager for IMU data
        mode (str): Angle unit mode - "degrees" (default) or "radians"
    
    Attributes:
        pos: Current position [x, y, z] in global frame
        velocity: Current velocity [vx, vy, vz] in global frame
        acceleration: Current acceleration [ax, ay, az] in global frame (gravity-compensated)
        orientation: Current orientation [yaw, pitch, roll]
    """
    
    def __init__(self, **kwargs):
        # Initialize State dataclass
        self.state = State(
            position=np.array(kwargs.get("position", [0.0, 0.0, 0.0])),
            orientation=np.array(kwargs.get("orientation", [0.0, 0.0, 0.0])),
            sensor_positions={
                "imu": np.array([0.0, 0.0, 0.0]),
                "lf_left": np.array([0.0, 0.0, 0.0]),
                "lf_right": np.array([0.0, 0.0, 0.0]),
                "color_sensor": np.array([0.0, 0.0, 0.0]),
                "cargo_deploy": np.array([0.0, 0.0, 0.0])
            }
        )

        # Previous state values for calculations
        self.prev_position = self.state.position.copy()
        self.prev_orientation = self.state.orientation.copy()

        # Sensor position offsets
        self.imu_to_ground = np.array([0.0, 0.0, -kwargs.get("imu_height", 0.015)])
        self.ground_to_lf_left = np.array([kwargs.get("imu_to_lf", 0.125), kwargs.get("lf_offset", 0.0225), kwargs.get("lf_height", 0.025)])
        self.ground_to_lf_right = np.array([kwargs.get("imu_to_lf", 0.125), -kwargs.get("lf_offset", 0.0225), kwargs.get("lf_height", 0.025)])
        self.ground_to_color = np.array([-kwargs.get("imu_to_color", 0.11), 0.0, kwargs.get("color_sensor_height", 0.025)])
        self.ground_to_cargo = np.array([-kwargs.get("imu_to_cargo", 0.24), 0.0, 0.0])

        # Calibration offsets (measured when stationary)
        self.state.bias = {
            "accel": np.array([0.0, 0.0, 0.0]),
            "gyro": np.array([0.0, 0.0, 0.0]),
            "mag": 0.0
        }

        # Velocity decay factor to reduce drift (0.0 = no decay, 1.0 = instant stop)
        self.velocity_decay = kwargs.get("velocity_decay", 0.4)
        
        # Threshold for considering acceleration as noise (m/s^2)
        self.accel_threshold = kwargs.get("accel_threshold", 0.05)
        
        # Motor velocity threshold for velocity decay (degrees/second)
        self.motor_velocity_threshold = kwargs.get("motor_velocity_threshold", 1.0)
        
        # Components
        self.transformer = Transformation(**kwargs)
        self.sensors = kwargs.get("sensors", None)
        self.motion_controller = kwargs.get("motion_controller", None)
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
        
        if self.sensors is None or not self.sensors.has_imu():
            print("No IMU sensor provided.")
            return False
        
        print("Calibrating IMU... Keep robot stationary.")
        
        accel_sum = np.array([0.0, 0.0, 0.0])
        gyro_sum = np.array([0.0, 0.0, 0.0])
        mag_sum = 0.0
        
        for i in range(samples):
            ax, ay, az = await self.sensors.get_accel()
            gx, gy, gz = await self.sensors.get_gyro()
            
            accel_sum += np.array([ax, ay, az])
            gyro_sum += np.array([gz, gy, gx])  # yaw, pitch, roll order
            mag_sum += await self.sensors.get_magnetic_magnitude()
            
            await asyncio.sleep(delay)
        
        # Average the readings
        self.state.bias["accel"] = accel_sum / samples
        self.state.bias["gyro"] = gyro_sum / samples
        self.state.bias["mag"] = mag_sum / samples
        
        # The Z acceleration bias should preserve gravity
        # We want to measure what "zero" acceleration looks like in the local frame
        # When stationary and level, accel should read (0, 0, ~9.81)
        # So we store the full reading and subtract it later, then add back gravity in global frame
        
        self.state.calibrated = True
        print(f"Calibration complete.")
        print(f"  Accel bias: {self.state.bias['accel']}")
        print(f"  Gyro bias: {self.state.bias['gyro']}")
        print(f"  Mag baseline: {self.state.bias['mag']:.2f} µT")
        
        return True
    
    async def update_orientation(self, **kwargs):
        """
        Update orientation by integrating gyroscope data.
        
        Args:
            dt (float): Time step in seconds
        """
        dt = kwargs.get("dt", 0.1)
        
        if self.sensors is None or not self.sensors.has_imu():
            print("No IMU sensor provided.")
            return False
        
        # Get angular velocity from gyroscope (degrees/second)
        d_roll, d_pitch, d_yaw = await self.sensors.get_gyro()
        angular_velocity = np.array([d_yaw, d_pitch, d_roll])

        # Subtract gyro bias if calibrated
        if self.state.calibrated:
            angular_velocity -= self.state.bias["gyro"]
        
        if not self.initialized:
            # First iteration: initialize rates without calculating acceleration
            self.state.angular_velocity = angular_velocity
            self.initialized = True
        else:
            # Integrate orientation using previous angular velocity
            self.state.orientation += 0.5 * self.state.angular_acceleration * (dt ** 2) + self.state.angular_velocity * dt
            self.state.angular_acceleration = (angular_velocity - self.state.angular_velocity) / dt
            self.state.angular_velocity = angular_velocity
        
        return True
    
    async def update_imu_position(self, **kwargs):
        """
        Update IMU position by integrating accelerometer data.
        
        Transforms local acceleration to global frame and subtracts gravity.
        Applies calibration bias and noise thresholding.
        The ground position is then calculated from the IMU position.
        
        Uses proper integration order:
        1. Update IMU position using previous velocity and acceleration
        2. Update velocity using previous acceleration
        3. Calculate new acceleration for next iteration
        4. Calculate ground position from IMU position
        
        Args:
            dt (float): Time step in seconds
            display (bool): Print position data if True
        """
        dt = kwargs.get("dt", 0.1)
        display = kwargs.get("display", False)
        
        if self.sensors is None or not self.sensors.has_imu():
            print("No IMU sensor provided.")
            return False
        
        # Store previous acceleration and velocity for integration
        prev_acceleration = self.state.acceleration.copy()
        prev_velocity = self.state.velocity.copy()
        
        # Update IMU position FIRST using previous velocity and acceleration
        # p = p0 + v0*dt + 0.5*a0*dt^2
        self.state.sensor_positions["imu"] = await self.transformer.translate_vector(
            vector=self.state.sensor_positions["imu"],
            translation=prev_velocity * dt + 0.5 * prev_acceleration * (dt ** 2)
        )
        
        # Calculate ground position from IMU position
        self.state.position = await self.transformer.translate_vector(
            vector=self.state.sensor_positions["imu"],
            translation=await self.transformer.rotate_vector(
                vector=self.imu_to_ground,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )
        
        # Update velocity SECOND using previous acceleration
        # v = v0 + a0*dt
        self.state.velocity = await self.transformer.translate_vector(
            vector=prev_velocity,
            translation=prev_acceleration * dt
        )
        
        # Apply velocity decay to reduce drift when motor is near stationary
        if self.motion_controller is not None:
            if abs(self.motion_controller.motor_velocity) < self.motor_velocity_threshold:
                self.state.velocity *= (1.0 - self.velocity_decay)
        else:
            # Fallback to acceleration threshold if no motion controller
            if np.linalg.norm(prev_acceleration) < self.accel_threshold:
                self.state.velocity *= (1.0 - self.velocity_decay)
        
        # NOW read new acceleration for next iteration
        ax, ay, az = await self.sensors.get_accel()
        accel_local = np.array([ax, ay, az])
        
        # Subtract calibration bias if calibrated
        if self.state.calibrated:
            accel_local -= self.state.bias["accel"]
        
        # Transform local acceleration to global frame
        accel_global = await self.transformer.rotate_vector(
            vector=accel_local,
            yaw=self.state.orientation[0],
            pitch=self.state.orientation[1],
            roll=self.state.orientation[2],
            invert=True
        )
        
        # If not calibrated, remove gravity (calibrated bias already includes gravity)
        if not self.state.calibrated:
            accel_global -= np.array([0.0, 0.0, GRAVITY])
        
        # Apply noise threshold - treat small accelerations as zero
        for i in range(3):
            if abs(accel_global[i]) < self.accel_threshold:
                accel_global[i] = 0.0
        
        # Store new acceleration for next iteration
        self.state.acceleration = accel_global
        
        if display:
            print(f"Position: {self.state.position}, Velocity: {self.state.velocity}, Acceleration: {self.state.acceleration}")
        
        return True
    
    async def update_positions_from_imu(self, **kwargs):
        """
        Update all sensor positions relative to the ground position.
        Ground position is already calculated in update_imu_position.
        """
        self.state.sensor_positions["lf_left"] = await self.transformer.translate_vector(
            vector=self.state.position,
            translation=await self.transformer.rotate_vector(
                vector=self.ground_to_lf_left,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )

        self.state.sensor_positions["lf_right"] = await self.transformer.translate_vector(
            vector=self.state.position,
            translation=await self.transformer.rotate_vector(
                vector=self.ground_to_lf_right,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )

        self.state.sensor_positions["color_sensor"] = await self.transformer.translate_vector(
            vector=self.state.position,
            translation=await self.transformer.rotate_vector(
                vector=self.ground_to_color,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )

        self.state.sensor_positions["cargo_deploy"] = await self.transformer.translate_vector(
            vector=self.state.position,
            translation=await self.transformer.rotate_vector(
                vector=self.ground_to_cargo,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )

    async def update_position(self, **kwargs):
        """
        Update position by integrating IMU data.
        
        Args:
            dt (float): Time step in seconds
            display (bool): Print position data if True
        """
        dt = kwargs.get("dt", 0.1)
        display = kwargs.get("display", False)
        
        await self.update_imu_position(dt=dt, display=display)
        await self.update_positions_from_imu()
        
        return True

    def get_position(self):
        """
        Get the current position as a tuple.
        
        Returns:
            tuple: Current position (x, y, z) in meters
        """
        return tuple(self.state.position)

    async def update(self, dt):
        """
        Update the location state based on sensor data.

        Args:
            dt (float): Time step in seconds.
        """
        if self.sensors and self.sensors.has_imu():
            imu_data = self.sensors.get_imu_data()
            self.state.acceleration_raw = imu_data["acceleration"]
            self.state.angular_velocity = imu_data["angular_velocity"]

            # Apply calibration bias
            self.state.acceleration = self.state.acceleration_raw - self.state.bias["accel"]

            # Integrate acceleration to update velocity and position
            self.state.velocity = (1 - self.velocity_decay) * (self.state.velocity + self.state.acceleration * dt)
            self.state.position += self.state.velocity * dt

            # Update orientation using angular velocity
            self.state.orientation += self.state.angular_velocity * dt

            # Apply thresholds to filter noise
            if np.linalg.norm(self.state.acceleration) < self.accel_threshold:
                self.state.acceleration = np.zeros(3)

            if np.linalg.norm(self.state.velocity) < self.motor_velocity_threshold:
                self.state.velocity = np.zeros(3)


class Navigation(Location):
    """
    3D navigation system.
    
    Extends Location to add magnetic field tracking and state updates.
    
    Args:
        Inherits all arguments from Location
    """
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    
    async def get_magnetic_field(self):
        """
        Read and return the magnetic field magnitude.
        
        Returns:
            float: Magnitude of magnetic field in micro-tesla
        """
        if self.sensors is None or not self.sensors.has_imu():
            return 0.0
        
        x_mag, y_mag, z_mag = await self.sensors.get_mag()
        self.state.magnetic_field = np.linalg.norm(np.array([x_mag, y_mag, z_mag]))
        
        return self.state.magnetic_field
    
    async def update_state(self, **kwargs):
        """
        Update full navigation state: position, orientation, and magnetic field.
        
        Args:
            dt (float): Time step in seconds (default: 0.1)
        """
        dt = kwargs.get("dt", 0.1)

        # Update previous state values
        self.prev_position = self.state.position.copy()
        self.prev_orientation = self.state.orientation.copy()

        # Update current state
        self.state.position = await self.update_position(dt=dt)
        self.state.orientation = await self.update_orientation(dt=dt)
        self.state.magnetic_field = await self.get_magnetic_field()

        # Update previous values in instance variables
        self.prev_magnetic_field = self.state.magnetic_field
    
    def print_state(self, timestamp, fields=None):
        """
        Print the current navigation state with timestamp.
        
        Args:
            timestamp (float): Current timestamp in seconds since start
            fields (list): List of fields to show. Options: "position", "velocity",
                          "acceleration", "orientation", "magnetic".
                          Use ["all"] for all fields, [] or None for nothing.
                          Default: ["all"]
        """
        if fields is None:
            fields = ["all"]
        
        if not fields or fields == []:
            return
        
        show_all = "all" in fields
        
        parts = [f"[{timestamp:.3f}s]"]
        
        if show_all or "position" in fields:
            position = tuple(round(p, 3) for p in self.state.position)
            parts.append(f"Pos: {position}")
        
        if show_all or "velocity" in fields:
            velocity = tuple(round(v, 3) for v in self.state.velocity)
            parts.append(f"Vel: {velocity}")
        
        if show_all or "acceleration" in fields:
            acceleration = tuple(round(a, 3) for a in self.state.acceleration)
            parts.append(f"Acc: {acceleration}")
        
        if show_all or "orientation" in fields:
            orientation = tuple(round(o, 2) for o in self.state.orientation)
            parts.append(f"Orient: {orientation}")
        
        if show_all or "magnetic" in fields:
            parts.append(f"Mag: {self.state.magnetic_field:.2f} µT")
        
        if len(parts) > 1:
            print(", ".join(parts))
    
    async def run_continuous_update(self, **kwargs):
        """
        Continuously update position at a fixed interval.
        
        Args:
            update_interval (float): Update interval in seconds (default: 0.1)
            print_state (bool): Whether to print state each iteration (default: False)
            calibrate (bool): Whether to calibrate IMU before starting (default: True)
            calibration_samples (int): Number of calibration samples (default: 50)
        """
        import time
        
        update_interval = kwargs.get("update_interval", 0.1)
        print_state_enabled = kwargs.get("print_state", False)
        do_calibrate = kwargs.get("calibrate", True)
        calibration_samples = kwargs.get("calibration_samples", 50)
        
        # Calibrate IMU if requested
        if do_calibrate and not self.state.calibrated:
            await self.calibrate(samples=calibration_samples)
        
        start_time = time.time()
        
        while True:
            await self.update_state(dt=update_interval)
            
            # Print current state if enabled
            if print_state_enabled:
                timestamp = time.time() - start_time
                self.print_state(timestamp)
            
            await asyncio.sleep(update_interval)


if __name__ == "__main__":
    # Example usage
    from systems.sensors import SensorInput
    
    sensors = SensorInput(imu=True)
    navigator = Navigation(sensors=sensors, mode="degrees")
    
    async def main():
        await navigator.run_continuous_update(
            update_interval=0.1,
            print_state=True,
            calibrate=True
        )
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopping navigation.")