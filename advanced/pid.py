import numpy as np


class PIDController:

    def __init__(self, kp=1, ki=0, kd=0):
        self.i = 0  # loop counter
        self.se = 0.0  # integrated error
        self.last_position = np.array([0, 0])
        self.se = 0
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def control_step(self, current_position, goal, dt):
        # KINEMATIC CONTROL
        error = goal - current_position
        d_position = current_position - self.last_position

        output = self.Kp * error + self.Ki * self.se * dt - self.Kd * d_position / dt

        self.se += error
        self.last_position = current_position

        return output
