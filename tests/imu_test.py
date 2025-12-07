from basehat import IMUSensor
import time
import math
print("IMU Successfully Imported")

imu = IMUSensor()
print("IMU Sensor found and initialized")

while True:
        ax, ay, az = imu.getAccel()
        gx, gy, gz = imu.getGyro()

        # Adjusting Acceleration
        theta = math.radians(gz)
        ax_adj = (ax * math.cos(theta)) - (ay * math.sin(theta))
        ay_adj = ax * math.sin(theta) + ay * math.cos(theta)

        x_mag, y_mag, z_mag = imu.getMag()
        print(f"theta: {theta} ax_adj: {ax_adj} ay_adj: {ay_adj}")
        # print(f"x_mag: {x_mag} y_mag: {y_mag} z_mag: {z_mag}")
        # print(f"ax: {ax} ay: {ay} az: {az} gx: {gx} gy: {gy} gz: {gz}")
        time.sleep(0.5)