import numpy as np


class IMUSimulator:
    def __init__(self, accel_noise=0.02, gyro_noise=0.5):
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise

    def read(self, theta, omega):
        """
        Simulate IMU readings based on true state.
        theta = angle (rad or deg depending on your system)
        omega = angular velocity
        """

        # Assume gravity vector for accel
        ax = -np.sin(theta)
        ay = 0.0
        az = np.cos(theta)

        # add noise
        ax += np.random.normal(0, self.accel_noise)
        ay += np.random.normal(0, self.accel_noise)
        az += np.random.normal(0, self.accel_noise)

        gx = omega + np.random.normal(0, self.gyro_noise)
        gy = 0.0 + np.random.normal(0, self.gyro_noise)
        gz = 0.0 + np.random.normal(0, self.gyro_noise)

        # convert to "raw-like" values if needed
        return ax, ay, az, gx, gy, gz