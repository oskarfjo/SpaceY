import numpy as np
import math
from Utils import functions as mu

class Regulator(object):
    def __init__(self):

        ### DYNAMIC VALUES ###
        self.theta_last = 0.0
        self.theta_m = 0.0


    def IMU(self, angle):
        noise = np.random.normal(0, 3)
        self.theta_m = angle + noise
        return self.theta_m
        
    def PD(self, theta, dt):
        self.IMU(theta) # generates a noisy theta measurement
        theta_m = theta

        error = -theta_m
        error_dot = - (theta_m - self.theta_last)/dt

        K_P = 1.5
        K_D = 0.6

        u = error*K_P + error_dot*K_D

        self.theta_last = theta_m

        u = mu.saturate(u, -8, 8)
        return u