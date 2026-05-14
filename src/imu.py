"""
Simple IMU simulator for quadcopter stabilization testing.

The simulator generates noisy accelerometer and gyroscope
measurements from the true simulated system state.
"""

import numpy as np

class IMUSimulator:
    def __init__(self, accel_noise=0.02, gyro_noise=0.5):
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise

    def read(self, theta, omega):
        """
        Simulate IMU readings based on true state.
        theta: radians
        omega: rad/s
        """

        # --- Accelerometer (gravity projection) ---
        ax = -np.sin(theta)
        ay = 0.0
        az = np.cos(theta)

        # Add noise (still in g's)
        ax += np.random.normal(0, self.accel_noise)
        ay += np.random.normal(0, self.accel_noise)
        az += np.random.normal(0, self.accel_noise)

        # Convert to raw IMU units
        ax *= 8192.0
        ay *= 8192.0
        az *= 8192.0

        # --- Gyroscope ---
        gx = np.degrees(omega) + np.random.normal(0, self.gyro_noise)
        gy = np.random.normal(0, self.gyro_noise)
        gz = np.random.normal(0, self.gyro_noise)

        # Convert to raw IMU units
        gx *= 65.5
        gy *= 65.5
        gz *= 65.5

        return ax, ay, az, gx, gy, gz