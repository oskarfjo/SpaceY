import numpy as np
import random
import math
from Utils import functions as mu

class Rocket(object):
    def __init__(self):

        ### DYNAMIC VALUES ###
        self.positiony  = 0
        self.positionx  = 0
        self.theta      = 0
        self.launched   = False
        self.velocityy  = 0.0
        self.velocityx  = 0.0

        ### PARAMETERS ###
        self.mass   = 1     # kg
        self.CD0    = 0.25
        self.area   = 0.04  # m2
        self.rho    = 1.225 # kgm-3
        self.g      = 9.81  # ms-2
        self.T      = 100    # N

    def step(self, dt):
        if self.launched:
            
            F_thrust = np.array([self.T * np.cos(self.theta), self.T * np.sin(self.theta)])
            
            F_gravity = np.array([0, -self.mass * self.g])

            velocity = np.array([self.velocityx, self.velocityy])
            speed = np.linalg.norm(velocity)

            if speed > 0:
                drag_direction = -velocity / speed
            else:
                drag_direction = np.array([0, 0])

            F_drag_magnitude = 0.5 * self.rho * self.CD0 * self.area * speed**2
            F_drag = F_drag_magnitude * drag_direction

            F_net = F_thrust + F_drag + F_gravity
            acceleration = F_net / self.mass

            self.velocityy += dt * acceleration[1]
            self.velocityx += dt * acceleration[0]

            self.positiony += dt * self.velocityy
            self.positionx += dt * self.velocityx

            self.positiony = mu.saturate(self.positiony, 0, 160)
            self.positionx = mu.saturate(self.positionx, -50, 50)

            if self.positiony == 0:
                self.velocityx = 0

            print(f'pos = [{self.positionx}, {self.positiony}], vel = [{self.velocityx}, {self.velocityy}]')

