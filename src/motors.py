def update_dynamics(theta, omega, u, dt, disturbance=0.0):
    """
    Simplified single-axis rotational dynamics:
    theta_dot = omega
    omega_dot = (torque - damping * omega) / inertia
    """

    # Physical parameters
    inertia = 1.0           # moment of inertia
    damping = 0.8           # rotational damping coefficient

    control_gain = 2.0      # control input treated as torque

    # Total torque
    torque = control_gain * u + disturbance

    # Compute angular acceleration
    omega_dot = (torque - damping * omega) / inertia

    # Semi-implicit Euler integration:
    # update angular velocity first, then update angle
    # using the new angular velocity value.
    omega += omega_dot * dt
    theta += omega * dt

    return theta, omega