#!/usr/bin/env python3
import math
import rospy

class PIDController:
    def __init__(self, p, i, d, setpoint):
        self.p = p
        self.i = i
        self.d = d
        self.integral = 0
        self.setpoint = setpoint
        self.prev_error = 0

    def calculate(self, measurement, dt):
        error = measurement - self.setpoint
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.p * error + self.i * self.integral + self.d * derivative