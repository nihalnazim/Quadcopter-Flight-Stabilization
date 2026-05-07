from dataclasses import dataclass


def clamp(x, min_val, max_val):
    return max(min_val, min(max_val, x))


@dataclass
class PDController:
    kp: float
    kd: float

    def update(self, error, rate=0.0):
        """
        PD control using measured angular rate instead of derivative of error.

        u = kp * error - kd * rate
        """
        return self.kp * error - self.kd * rate


@dataclass
class ControlState:
    roll: PDController
    pitch: PDController
    setpoint_roll: float = 0.0
    setpoint_pitch: float = 0.0
    base_throttle: float = 0.5


def create_default_controller():
    return ControlState(
        roll=PDController(kp=2.0, kd=0.8),
        pitch=PDController(kp=2.0, kd=0.8),
        base_throttle=0.5
    )


def compute_control(ctrl, attitude, dt):
    error_roll = ctrl.setpoint_roll - attitude.roll
    error_pitch = ctrl.setpoint_pitch - attitude.pitch

    rate_roll = getattr(attitude, "roll_rate", 0.0)
    rate_pitch = getattr(attitude, "pitch_rate", 0.0)

    u_roll = ctrl.roll.update(error_roll, rate_roll)
    u_pitch = ctrl.pitch.update(error_pitch, rate_pitch)

    corr_roll = clamp(u_roll, -0.2, 0.2)
    corr_pitch = clamp(u_pitch, -0.2, 0.2)

    t = ctrl.base_throttle

    m1 = clamp(t + corr_pitch + corr_roll, 0.0, 1.0)
    m2 = clamp(t + corr_pitch - corr_roll, 0.0, 1.0)
    m3 = clamp(t - corr_pitch + corr_roll, 0.0, 1.0)
    m4 = clamp(t - corr_pitch - corr_roll, 0.0, 1.0)

    return m1, m2, m3, m4


def compute_single_axis_control(ctrl, attitude, dt):
    error = ctrl.setpoint_roll - attitude.roll
    rate = getattr(attitude, "roll_rate", 0.0)

    u = ctrl.roll.update(error, rate)

    # Prevent unrealistic control spikes in the simplified model
    u = clamp(u, -0.5, 0.5)

    return u