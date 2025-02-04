import numpy as np
import random
import math
from Utils import functions as mu

DEBUG = True

class Rocket(object):
    def __init__(self):

        ### DYNAMIC VALUES ###
        self.positionY  = 0
        self.positionX  = 0
        self.theta      = 0
        self.launched   = False
        self.velocityY  = 0.0
        self.velocityX  = 0.0

        ### PARAMETERS ###
        self.mass   = 0.6027    # kg
        self.CD0    = 0.25
        self.area   = 0.004     # m2
        self.rho    = 1.225     # kgm-3
        self.g      = 9.81      # ms-2
        self.T      = 30       # N


    def dynamics_step(self, dt):
        if True:
            
            ### Simulating dynamics ###


            # adds the thrust force only when the rocket is in the launched state
            if self.launched:
                F_thrust = np.array([self.T * np.sin(np.deg2rad(self.theta)), self.T * np.cos(np.deg2rad(self.theta))])
            else:
                F_thrust = 0

            F_gravity = np.array([0, -self.mass * self.g])

            velocity = np.array([self.velocityX, self.velocityY]) # vector for velocity in 2D

            F_drag = mu.compute_drag(velocity,
                      self.theta,         # deg
                      self.CD0,           # baseline
                      self.rho,
                      A_front=0.0044,     # from diameter = 75mm
                      A_side=0.0375       # side area from height = 50 cm
                      )

            F_sum = F_thrust + F_drag + F_gravity 
            acceleration = F_sum / self.mass # F = ma  ->  F/m = a

            # integrating acceleration for velocity
            self.velocityY += dt * acceleration[1]
            self.velocityX += dt * acceleration[0]

            # integrating velocity for position
            self.positionY += dt * self.velocityY
            self.positionX += dt * self.velocityX


                ### QOL ###

            # makes borders at the edges of the screen so that the rocket cant go out of sight
            self.positionY = mu.saturate(self.positionY, 0, 122)
            self.positionX = mu.saturate(self.positionX, -38, 38)

            # Stops the rocket from simulating movement when it is at the borders
            if self.positionY == 0 or self.positionY == 122:
                self.velocityX = 0
                self.velocityY = 0

            if DEBUG:
                print(f'pos = [{self.positionX}, {self.positionY}]m, vel = [{self.velocityX}, {self.velocityY}] ms-1')
                print(f'acceleration = {acceleration} ms-2')
                print(f'F_drag = {F_drag} N, F_thrust = {F_thrust} N, F_net = {F_sum} N')

