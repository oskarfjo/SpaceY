import numpy as np
import math
import random

class functions:

    def noise(sd):
        return np.random.normal(0, sd)
    
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