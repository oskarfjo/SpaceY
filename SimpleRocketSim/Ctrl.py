import numpy as np
import math
from Utils import functions as mu

class Regulator(object):
    def __init__(self):

        ### DYNAMIC VALUES ###
        self.theta_last = 0.0

        ### PARAMS ###
        
        
    def PD(self, theta, dt):
        error = -theta
        error_dot = - (theta - self.theta_last)/dt

        K_P = 1.5
        K_D = 1

        u = error*K_P + error_dot*K_D

        self.theta_last = theta

        u = mu.saturate(u, -8, 8)
        return u