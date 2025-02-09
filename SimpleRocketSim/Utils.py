import numpy as np
import math
import random

class functions:

    def moment_inertia(mass, Sd, Sh): # , S
        I_S = mass * (Sd/2)**2 / 4 + mass * Sh**2 / 12 # + mass * Sl**2
        I_C = 0 #
        return 0.3 * I_S + I_C

    def noise(ex, sd):
        return np.random.normal(ex, sd)
    
    def sig_test(sig):
        if math.isnan(sig):
            test_result = False
        else:
            test_result = True
        return test_result
    
    def rps_test(rps, max_rps):   
        if rps > max_rps:
            rps = max_rps
        elif rps < -max_rps:
            rps = -max_rps
        else:
            rps = rps
        return rps

    def saturate(val, min, max):
        if val >= max:
            A = max
        elif val <= min:
            A = min
        else:
            A = val
        return A
        
    def mapToPiPi(angle_in_radians):
        # Wrap the angle to the range [-pi, pi]
        wrapped_angle = (angle_in_radians + math.pi) % (2 * math.pi) - math.pi
        
        # This adjustment is necessary to handle the case when angle + pi == 2 * pi
        if wrapped_angle <= -math.pi:
            wrapped_angle += 2 * math.pi
        
        return wrapped_angle

    def mapToZero2Pi(angle_in_radians):
        return angle_in_radians % (2 * np.pi)


    def mapToDeg(angle_in_degrees):
        # Wrap the angle to the range [-180, 180]
        wrapped_angle = (angle_in_degrees + 180) % (2 * 180) - 180
        
        # This adjustment is necessary to handle the case when angle + 180 == 2 * 180
        if wrapped_angle <= -180:
            wrapped_angle += 2 * 180
        
        return wrapped_angle




# ---------------
# ---- KLADD ----
# ---------------

"""
Bingus

"""