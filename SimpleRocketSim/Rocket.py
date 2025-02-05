import numpy as np
import random
import math
from Utils import functions as mu

DEBUG = True

class Rocket(object):
    def __init__(self):

        ### DYNAMIC VALUES ###
        self.positionZ  = 0
        self.positionX  = 0
        self.theta      = 0
        self.launched   = False
        self.velocityZ  = 0.0
        self.velocityX  = 0.0
        self.clock      = 0.0
        self.time_step  = 0.0
        self.n          = 0

        ### MOTORS ###
        self.cluster    = 3
        self.burn_time = 2.1 # s
        self.D9 = np.array([5, 20, 12, 8.7, 8.7, 8.7, 8.7, 8.7, 8.7, 8.7, 8.7]) # n = 0,25s

        ### PARAMETERS ###
        self.mass       = 1.3       # kg
        self.CD0        = 0.25
        self.rho        = 1.225     # kgm-3
        self.g          = 9.81      # ms-2
        self.diameter   = 75e-3     # xx mm to 0,0xx m
        self.length     = 50e-2     # xx cm to 0.xx m

    def timer(self, dt):
        if self.launched:
            self.clock += dt
        else:
            self.clock = 0.0
        return

    def thrust_magnitude(self, array, time):
        if time >= self.time_step + 0.25:
            self.n +=1
            self.time_step += 0.25
        elif time > self.burn_time:
            self.n = 0
            self.time_step = 0
            self.launched = False
        return array[self.n] # gives the thrust a dynamic magnitude in accordance to the datasheet

    def drag(self, velocity):
        speed = np.linalg.norm(velocity) # gives the magnitude of the velocity vector
        if speed > 0:
            drag_direction = -velocity / speed # defines a unit vector for the drag force direction that is opposite the velocity
        else:
            drag_direction = np.array([0, 0]) # zero drag when at rest
        area = 0.25 * np.pi * self.diameter**2 # m2
        F_drag_magnitude = 0.5 * self.rho * self.CD0 * area * speed**2 # calculates the scalar magnitude of the drag force. kgms-2
        return F_drag_magnitude * drag_direction # returns the drag as a vector
    
    def thrust(self, angle):
        force = self.thrust_magnitude(self.cluster*self.D9, self.clock) # calculates the total magnitude of the thrust force at the current time
        return np.array([force * np.sin(np.deg2rad(angle)), force * np.cos(np.deg2rad(angle))]) # calculates thrust on the rocket in relation of the gimbal angle

    def dynamics_step(self, dt):
            ### Simulating dynamics ###
        if True:

            # adds the thrust force only when the rocket is in the launched state
            if self.launched:
                F_thrust = self.thrust(self.theta)
            else:
                F_thrust = np.array([0, 0])
                self.clock = 0.0

            F_gravity = np.array([0, -self.mass * self.g])

            velocity = np.array([self.velocityX, self.velocityZ]) # vector for velocity in 2D
            F_drag = self.drag(velocity)

            F_sum = F_thrust + F_drag + F_gravity 
            acceleration = F_sum / self.mass # F = ma  ->  F/m = a

            ### integrating acceleration for velocity ###
            self.velocityZ += dt * acceleration[1]
            self.velocityX += dt * acceleration[0]

            ### integrating velocity for position ###
            self.positionZ += dt * self.velocityZ
            self.positionX += dt * self.velocityX


                ### QOL ###

            # makes borders at the edges of the screen so that the rocket cant go out of sight
            self.positionZ = mu.saturate(self.positionZ, 0, 122)
            self.positionX = mu.saturate(self.positionX, -38, 38)

            # Stops the rocket from simulating movement when it is at the borders
            if self.positionZ == 0 or self.positionZ == 122:
                self.velocityX = 0
                self.velocityZ = 0

            if DEBUG:
                print(f'position = [{self.positionX}, {self.positionZ}]m')
                print(f'velocity = [{self.velocityX}, {self.velocityZ}ms-1')
                print(f'acceleration = {acceleration} ms-2')
                print(f'F_drag = {F_drag} N, F_thrust = {F_thrust} N, F_net = {F_sum} N')
                print(f'TTW: {round(abs(F_thrust[1] / (F_gravity[1])), 2)}')

