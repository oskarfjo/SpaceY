import numpy as np
import math
import random

class functions:

    def moment_inertia(mass, Sd, Sh): # , S
        I_S = mass * (Sd/2)**2 / 4 + mass * Sh**2 / 12 # + mass * Sl**2
        I_C = 0 #
        return 0.3* I_S + I_C

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
    




# ---------------
# ---- KLADD ----
# ---------------

"""
Bingus

"""