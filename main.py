from src.attitude import AttitudeState
from src.control import create_default_controller, compute_single_axis_control
from src.safety import SafetySystem
from src.motors import update_dynamics
from src.imu import IMUSimulator
from src.calibration import calibrate_gyro
import math


def run_simulation(t_final=5.0, dt=1/250):
    attitude = AttitudeState()
    controller = create_default_controller()
    safety = SafetySystem(max_angle=45.0)
    imu = IMUSimulator()

    safety.arm()
    "calibrate_gyro(attitude, imu, sample_count=200)"

    theta = 0.3
    omega = 0.0

    time_log = []
    theta_log = []
    omega_log = []
    motor_log = []

    n_steps = int(t_final / dt)

    for k in range(n_steps):
        t = k * dt

        """ax, ay, az, gx, gy, gz = imu.read(theta=theta, omega=omega)
        attitude.update(ax, ay, az, gx, gy, gz, dt)"""

        attitude.roll = math.degrees(theta)
        attitude.pitch = 0.0

        allowed = safety.check(attitude.roll, attitude.pitch)

        if allowed:
            m1, m2, m3, m4 = compute_control(controller, attitude, dt)
            scale = safety.throttle_scale()
            m1, m2, m3, m4 = m1 * scale, m2 * scale, m3 * scale, m4 * scale
        else:
            m1 = m2 = m3 = m4 = 0.0

        theta, omega = update_dynamics(theta, omega, m1, m2, m3, m4, dt)

        time_log.append(t)
        theta_log.append(theta)
        omega_log.append(omega)
        motor_log.append((m1, m2, m3, m4))

    return time_log, theta_log, omega_log, motor_log

if __name__ == "__main__":
    time_log, theta_log, omega_log, motor_log = run_simulation()
    print("Simulation complete.")
    print(f"Time steps: {len(time_log)}")
    print(f"Initial theta: {theta_log[0]}")
    print(f"Final theta: {theta_log[-1]}")