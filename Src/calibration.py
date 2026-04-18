def calibrate_gyro(attitude_state, imu, sample_count=500, dt=0.002):
    """
    Estimate gyro bias by averaging readings while stationary.
    """

    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0

    print("Starting gyro calibration... keep system still.")

    for _ in range(sample_count):
        ax, ay, az, gx, gy, gz = imu.read(theta=0.0, omega=0.0)

        sum_x += gx
        sum_y += gy
        sum_z += gz

    avg_x = sum_x / sample_count
    avg_y = sum_y / sample_count
    avg_z = sum_z / sample_count

    attitude_state.set_gyro_bias(avg_x, avg_y, avg_z)

    print(f"Gyro bias: X={avg_x:.3f}, Y={avg_y:.3f}, Z={avg_z:.3f}")