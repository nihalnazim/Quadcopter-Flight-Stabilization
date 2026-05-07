from src.attitude import AttitudeState
from src.control import create_default_controller, compute_single_axis_control
from src.safety import SafetySystem
from src.motors import update_dynamics
from src.imu import IMUSimulator

import math
import matplotlib.pyplot as plt


def run_simulation(t_final=5.0, dt=1/250, use_imu=False):
    attitude = AttitudeState()
    controller = create_default_controller()
    safety = SafetySystem(max_angle=math.radians(45.0))
    imu = IMUSimulator(accel_noise=0.02, gyro_noise=0.5)

    safety.arm()

    theta = 0.3
    omega = 0.0

    attitude.roll = theta
    attitude.pitch = 0.0

    time_log = []
    theta_log = []
    measured_theta_log = []
    omega_log = []
    motor_log = []

    n_steps = int(t_final / dt)
    disturbance_applied = False

    for k in range(n_steps):
        t = k * dt

        # Apply disturbance at t = 1.0 s
        if not disturbance_applied and t >= 1.0:
            theta += 0.08
            disturbance_applied = True

        # --- Measurement pipeline ---
        # Use IMU (noisy measurement) or direct state (ideal measurement)
        if use_imu:
            ax, ay, az, gx, gy, gz = imu.read(theta, omega)
            attitude.update(ax, ay, az, gx, gy, gz, dt)
        else:
            attitude.roll = theta
            attitude.roll_rate = omega
            attitude.pitch = 0.0
            attitude.pitch_rate = 0.0

        allowed = safety.check(attitude.roll, attitude.pitch)

        # Compute control only if within safe operating limits
        if allowed:
            u = compute_single_axis_control(controller, attitude, dt)
            u *= safety.throttle_scale()
            u = max(min(u, 5.0), -5.0)
        else:
            u = 0.0

        theta, omega = update_dynamics(theta, omega, u, dt)

        time_log.append(t)
        theta_log.append(theta)
        measured_theta_log.append(attitude.roll)
        omega_log.append(omega)
        motor_log.append(u)

    return time_log, theta_log, measured_theta_log, omega_log, motor_log


if __name__ == "__main__":
    # Change this to True to use noisy IMU measurements
    use_imu = True

    time_log, theta_log, measured_theta_log, omega_log, motor_log = run_simulation(
        use_imu=use_imu
    )

    print("Simulation complete.")
    print(f"IMU enabled: {use_imu}")
    print(f"Time steps: {len(time_log)}")
    print(f"Initial theta: {theta_log[0]}")
    print(f"Final theta: {theta_log[-1]}")

    plt.figure()
    plt.plot(time_log, theta_log, label="True theta")
    plt.plot(time_log, measured_theta_log, label="Measured theta", alpha=0.7)
    plt.xlabel("Time (s)")
    plt.ylabel("Theta (rad)")
    plt.title("Single-Axis Stabilization with IMU Measurement")
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(time_log, omega_log)
    plt.xlabel("Time (s)")
    plt.ylabel("Angular velocity (rad/s)")
    plt.title("Angular Velocity vs Time")
    plt.grid()

    plt.figure()
    plt.plot(time_log, motor_log)
    plt.xlabel("Time (s)")
    plt.ylabel("Control input u")
    plt.title("Control Input vs Time")
    plt.grid()

    plt.show()