"""Small PID controller based on the supplied wafer_cart/pid.py."""


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.derivative = 0.0

    def update(self, error, dt):
        dt = max(0.001, dt)
        self.integral = max(-2.0, min(2.0, self.integral + error * dt))
        raw_d = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.derivative = 0.8 * self.derivative + 0.2 * raw_d
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * self.derivative
