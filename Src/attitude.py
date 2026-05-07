import math
from dataclasses import dataclass

GYRO_SENSITIVITY = 65.5       # ±500 deg/s
ACCEL_SENSITIVITY = 8192.0    # ±4 g
COMPLEMENTARY_ALPHA = 0.95


@dataclass
class AttitudeState:
    roll: float = 0.0      # radians in simulation mode
    pitch: float = 0.0     # radians in simulation mode

    roll_rate: float = 0.0
    pitch_rate: float = 0.0

    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    gyro_bias_z: float = 0.0

    calibrated: bool = False

    def set_gyro_bias(self, bx: float, by: float, bz: float) -> None:
        self.gyro_bias_x = bx
        self.gyro_bias_y = by
        self.gyro_bias_z = bz
        self.calibrated = True

    def update(
        self,
        ax_raw: int,
        ay_raw: int,
        az_raw: int,
        gx_raw: int,
        gy_raw: int,
        gz_raw: int,
        dt: float,
    ) -> None:
        
        # Convert raw accelerometer values to normalized units
        ax = ax_raw / ACCEL_SENSITIVITY
        ay = ay_raw / ACCEL_SENSITIVITY
        az = az_raw / ACCEL_SENSITIVITY

        # Convert gyro readings and remove bias
        gx = gx_raw / GYRO_SENSITIVITY - self.gyro_bias_x
        gy = gy_raw / GYRO_SENSITIVITY - self.gyro_bias_y
        gz = gz_raw / GYRO_SENSITIVITY - self.gyro_bias_z

        # Store angular rates (converted to radians)
        self.roll_rate = math.radians(gx)
        self.pitch_rate = math.radians(gy)

        _ = gz

        # Estimate roll from accelerometer (gravity vector)
        roll_acc = math.atan2(-ax, az)

        # Estimate pitch from accelerometer
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Predict angle using gyro integration
        roll_gyro = self.roll + self.roll_rate * dt
        pitch_gyro = self.pitch + self.pitch_rate * dt

        # Complementary filter: combine gyro (smooth) and accel (absolute reference)
        self.roll = (
            COMPLEMENTARY_ALPHA * roll_gyro
            + (1.0 - COMPLEMENTARY_ALPHA) * roll_acc
        )
        self.pitch = (
            COMPLEMENTARY_ALPHA * pitch_gyro
            + (1.0 - COMPLEMENTARY_ALPHA) * pitch_acc
        )