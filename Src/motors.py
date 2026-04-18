def update_dynamics(theta, omega, m1, m2, m3, m4, dt):
    """
    Very simplified drone pitch/roll dynamics.
    """

    # Simplified torque model
    torque = (m1 + m3) - (m2 + m4)

    # Constants (tunable)
    inertia = 1.0
    damping = 0.8
    control_gain = 2.0

    # Angular acceleration
    omega_dot = (control_gain * torque - damping * omega) / inertia

    # Update state
    theta_dot = omega

    theta += theta_dot * dt
    omega += omega_dot * dt

    return theta, omega