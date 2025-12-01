import numpy as np
import sys
import time
from ..modules.basehat.imu_sensor import IMUSensor
import math

class Location:
    def __init__(self, x: float, y: float, imu: IMUSensor):
        self.x0 = x
        self.y0 = y
        self.t0 = 0.0  # time in seconds
        self.l_rear = 0.0  # distance to rear axle
        self.l_front = 0.0  # distance to front axle
        self.imu = imu
        self.coord_log = ([[x, y], self.t])  # log of coordinates and time
        self.imu_log = []  # log of IMU readings
    
    def convert_to_polar(self, x: float, y: float):
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
        return r, theta

    def convert_to_cartesian(self, r: float, theta: float):
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y
    
    def update_position(self, v0_x: float, v0_y: float):
        # Get acceleration data from IMU
        ax_mes, ay_mes, az_mes = self.imu.getAccel()
        yz_gyro, xz_gyro, xy_gyro = self.imu.getGyro()

        # Time update
        t1 = time.time()
        dt = t1 - self.t0
        self.t0 = t1

        # Adjust acceleration for vehicle geometry
        ax = ax_mes + math.radians(xy_gyro) * self.l_rear

    
