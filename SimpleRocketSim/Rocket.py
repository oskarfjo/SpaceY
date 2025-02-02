import numpy as np
import random
import math
from Utils import functions as mu

DEBUG = True

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
        self.CD0    = 0.025
        self.area   = 0.04  # m2
        self.rho    = 1.225 # kgm-3
        self.g      = 9.81  # ms-2
        self.T      = 20    # N

    def step(self, dt):
        if True:
            
            ### Simulating dynamics ###

            # adds the thrust force when the rocket is in the launched state
            if self.launched:
                F_thrust = np.array([self.T * np.sin(np.deg2rad(self.theta)), self.T * np.cos(np.deg2rad(self.theta))])
            else:
                F_thrust = 0

            F_gravity = np.array([0, -self.mass * self.g])

            velocity = np.array([self.velocityx, self.velocityy]) # vector for velocity in 2D
            speed = np.linalg.norm(velocity)

            # the drag works opposite to the direction of the velocity
            if speed > 0:
                drag_direction = -velocity / speed
            else:
                drag_direction = np.array([0, 0])

            F_drag_magnitude = 0.5 * self.rho * self.CD0 * self.area * speed**2
            F_drag = F_drag_magnitude * drag_direction

            F_net = F_thrust + F_drag + F_gravity # vector in 2D
            acceleration = F_net / self.mass # vector in 2D

            # integrating acceleration for velocity
            self.velocityy += dt * acceleration[1]
            self.velocityx += dt * acceleration[0]

            # integrating velocity for position
            self.positiony += dt * self.velocityy
            self.positionx += dt * self.velocityx


                ### QOL ###

            # makes borders at the edges of the screen so that the rocket cant go out of sight
            self.positiony = mu.saturate(self.positiony, 0, 1600)
            self.positionx = mu.saturate(self.positionx, -500, 500)

            # Stops the rocket from simulating movement when it is at the borders
            if self.positiony == 0 or self.positiony == 1600:
                self.velocityx = 0
                self.velocityy = 0

            if DEBUG:
                print(f'pos = [{self.positionx}, {self.positiony}], vel = [{self.velocityx}, {self.velocityy}]')
                print(f'acceleration = {acceleration}')

