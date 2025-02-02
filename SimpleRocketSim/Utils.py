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
    
        ### GPT generated code for calculating the drag of the rocket while accounting for different areas and geometric coefficients ###
    def compute_drag(velocity, rocket_angle_degs, CD0, rho, A_front, A_side):
        speed = np.linalg.norm(velocity)
        if speed == 0:
            return np.array([0.0, 0.0])

        # 1) Rocket axis (unit vector) from angle
        theta_rad = math.radians(rocket_angle_degs)
        rocket_axis = np.array([math.sin(theta_rad), math.cos(theta_rad)])

        # 2) Velocity direction (unit vector)
        vel_dir = velocity / speed

        # 3) Angle of attack alpha
        dot_r_v = np.dot(rocket_axis, vel_dir)
        # Clip dot for numerical stability: -1 <= dot_r_v <= 1
        dot_r_v = max(min(dot_r_v, 1.0), -1.0)
        alpha = math.acos(dot_r_v)

        # 4) Effective area: mix between front area and side area
        A_eff = A_front * (math.cos(alpha))**2 + A_side * (math.sin(alpha))**2

        # 5) Standard drag magnitude
        F_drag_mag = 0.5 * rho * CD0 * A_eff * (speed**2)

        # 6) Drag direction: opposite to velocity
        drag_dir = -vel_dir

        return F_drag_mag * drag_dir
    




# ---------------
# ---- KLADD ----
# ---------------

"""
                # the drag works opposite to the direction of the velocity
            if speed > 0:
                drag_direction = -velocity / speed
            else:
                drag_direction = np.array([0, 0])

            F_drag_magnitude = 0.5 * self.rho * self.CD0 * self.area * speed**2
            F_drag = F_drag_magnitude * drag_direction

"""