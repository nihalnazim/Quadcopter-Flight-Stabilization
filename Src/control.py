from dataclasses import dataclass


def clamp(x, min_val, max_val):
    return max(min_val, min(max_val, x))


@dataclass
class PDController:
    kp: float
    kd: float
    prev_error: float = 0.0

    def update(self, error, dt):
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        self.prev_error = error
        return self.kp * error + self.kd * derivative


@dataclass
class ControlState:
    roll: PDController
    pitch: PDController

    setpoint_roll: float = 0.0
    setpoint_pitch: float = 0.0

    base_throttle: float = 0.5


def create_default_controller():
    return ControlState(
        roll=PDController(kp=3.0, kd=0.5),
        pitch=PDController(kp=3.0, kd=0.5),
        base_throttle=0.5
    )


def compute_control(ctrl, attitude, dt):
    # errors
    error_roll = ctrl.setpoint_roll - attitude.roll
    error_pitch = ctrl.setpoint_pitch - attitude.pitch

    # PD outputs
    u_roll = ctrl.roll.update(error_roll, dt)
    u_pitch = ctrl.pitch.update(error_pitch, dt)

    # clamp corrections (VERY important for stability)
    corr_roll = clamp(u_roll, -0.2, 0.2)
    corr_pitch = clamp(u_pitch, -0.2, 0.2)

    t = ctrl.base_throttle

    # motor mixing (same as C)
    m1 = clamp(t + corr_pitch + corr_roll, 0.0, 1.0)
    m2 = clamp(t + corr_pitch - corr_roll, 0.0, 1.0)
    m3 = clamp(t - corr_pitch + corr_roll, 0.0, 1.0)
    m4 = clamp(t - corr_pitch - corr_roll, 0.0, 1.0)

    return m1, m2, m3, m4

def compute_single_axis_control(ctrl, attitude, dt):
    error = ctrl.setpoint_roll - attitude.roll
    u = ctrl.roll.update(error, dt)
    return u