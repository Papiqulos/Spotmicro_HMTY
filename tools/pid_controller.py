class PIDController:

    def __init__(self, kp, ki, kd, max_integral=0.7):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral

        self.previous_error = 0
        self.integral_sum = 0
        self.first_run = True

    def update(self, error, dt):
        if self.first_run:
            self.previous_error = error
            self.first_run = False

        proportional = self.kp * error

        self.integral_sum += error * dt
        if self.max_integral is not None:
            self.integral_sum = max(-self.max_integral, min(self.max_integral, self.integral_sum))
        integral = self.ki * self.integral_sum

        derivative = self.kd * (error - self.previous_error) / dt if dt > 0 else 0.0

        self.previous_error = error

        return proportional + derivative + integral

    def reset(self):
        self.integral_sum = 0
        self.previous_error = 0
        self.first_run = True
