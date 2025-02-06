import numpy as np
import random
import math
from Utils import functions as mu

DEBUG = True

class Rocket(object):
    def __init__(self):

        ### DYNAMIC VALUES ###
        self.positionZ  = 0 # m
        self.positionX  = 0 # m
        self.theta      = 0 # deg
        self.theta_last = 0 # deg
        self.alpha      = 0 # deg
        self.launched   = False
        self.velocityZ  = 0.0   # m/s
        self.velocityX  = 0.0   # m/s
        self.acceleration = 0.0 # m/s2
        self.clock      = 0.0   # s
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

        self.thruster_pos   = 0.25  # m from cm
        self.cop_pos        = 0.1   # m from cm

    def timer(self, dt):
        if self.launched:
            self.clock += dt
        else:
            self.clock = 0.0
        return

    ## takes the angle of the gimbal and calculates a new theta from the torque and inertia ##
    def computed_rocket_angle(self, alpha, thrust, drag, dt):
        thrust_mag = np.linalg.norm(thrust)
        drag_mag = np.linalg.norm(drag)
        F = thrust_mag*self.thruster_pos*np.sin(np.deg2rad(alpha)) + drag_mag*self.cop_pos*np.sin(np.deg2rad(self.theta))
        I = mu.moment_inertia(self.mass, self.diameter, self.length)
        theta_next = F * dt**2/I + 2*self.theta - self.theta_last # computes theta[n+1]
        if DEBUG:
            print(f'theta[n-1] = {round(self.theta_last, 2)}, theta[n] = {round(self.theta, 2)}, theta[n+1] = {round(theta_next, 2)}')
        self.theta_last = self.theta # updates theta[n-1]
        self.theta = theta_next # updates theta[n]

    ## gives the thrust a dynamic magnitude in accordance to the datasheet ##
    def thrust_magnitude(self, array, time):
        if time > self.burn_time:
            self.n = 0
            self.time_step = 0.0
            self.launched = False
            return 0
        elif time >= self.time_step + 0.25: # jumps to the next val in the array every 0,25s
            self.n +=1
            self.time_step += 0.25
        return array[self.n]

    ## calculates the drag that opposes the velocity vector ##
    def drag(self, velocity):
        speed = np.linalg.norm(velocity)
        if speed > 0:
            drag_direction = -velocity / speed
        else:
            drag_direction = np.array([0, 0])
        area = 0.25 * np.pi * self.diameter**2
        F_drag_magnitude = 0.5 * self.rho * self.CD0 * area * speed**2
        return F_drag_magnitude * drag_direction # returns the drag as a vector
    
    ## calculates the total magnitude of the thrust force at the current time ##
    def thrust(self, angle):
        force = self.thrust_magnitude(self.cluster*self.D9, self.clock) # dynamic scalar force in Newtons
        return np.array([force * np.sin(np.deg2rad(angle)), force * np.cos(np.deg2rad(angle))])

    def dynamics_step(self, dt):
            ### Simulating dynamics ###
        if True:

            # adds the thrust force only when the rocket is in the launched state
            if self.launched:
                F_thrust = self.thrust(self.theta + self.alpha) # thrusts in a direction that combines the angle of the rocket and the gimbal angle
            else:
                F_thrust = np.array([0, 0])
                self.clock = 0.0

            F_gravity = np.array([0, -self.mass * self.g]) # vector for gravity in 2D

            velocity = np.array([self.velocityX, self.velocityZ]) # vector for velocity in 2D
            F_drag = self.drag(velocity) # vector for drag in 2D

            F_sum = F_thrust + F_drag + F_gravity
            acceleration = F_sum / self.mass # F = ma  ->  F/m = a
            self.acceleration = np.linalg.norm(acceleration) # scalar acceleration used for displaying g's in the sim

            ### integrating acceleration for velocity ###
            self.velocityZ += dt * acceleration[1]
            self.velocityX += dt * acceleration[0]

            ### integrating velocity for position ###
            self.positionZ += dt * self.velocityZ
            self.positionX += dt * self.velocityX

            ### computes the angle theta[n + 1] ###
            if self.positionZ > 0:
                self.computed_rocket_angle(self.alpha, F_thrust, F_drag, dt)

                ### QOL ###

            # makes borders at the edges of the screen so that the rocket cant go out of sight
            self.positionZ = mu.saturate(self.positionZ, 0, 122)
            self.positionX = mu.saturate(self.positionX, -59, 59)

            # Stops the rocket from simulating movement when it is at the borders
            if self.positionZ == 0 or self.positionZ == 122:
                self.velocityX = 0
                self.velocityZ = 0

            if DEBUG:
                print(f'position = [{round(self.positionX, 2)}, {round(self.positionZ, 2)}]m')
                print(f'velocity = [{round(self.velocityX, 2)}, {round(self.velocityZ, 2)}]ms-1')
                print(f'acceleration = [{round(acceleration[0], 2)}, {round(acceleration[1], 2)}]ms-2')
                print(f'F_drag = [{round(F_drag[0], 2)}, {round(F_drag[1], 2)}] N, F_thrust = [{round(F_thrust[0], 2)}, {round(F_thrust[1], 2)}] N, F_net = [{round(F_sum[0], 2)}, {round(F_sum[1], 2)}] N')
                print(f'TTW: {round(abs(F_thrust[1] / (F_gravity[1])), 2)}')
                print(f'angle = {self.alpha}')

