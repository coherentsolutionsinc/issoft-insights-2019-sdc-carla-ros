from math import atan2, sin

class Stanley(object):
    def __init__(self, max_angle, k):
        self.max_angle = max_angle
        self.k = k
        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, yaw, v, e):
       
        val =  yaw + atan2(self.k * e, v)

        val = max(min(val, self.max_angle), -self.max_angle)

        self.last_error = e

        return val