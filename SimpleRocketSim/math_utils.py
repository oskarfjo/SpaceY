import numpy as np
import math

def mapToPiPi(angle_in_radians):
    # Wrap the angle to the range [-pi, pi]
    wrapped_angle = (angle_in_radians + math.pi) % (2 * math.pi) - math.pi
    
    # This adjustment is necessary to handle the case when angle + pi == 2 * pi
    if wrapped_angle <= -math.pi:
        wrapped_angle += 2 * math.pi
    
    return wrapped_angle

def mapToZero2Pi(angle_in_radians):
    return angle_in_radians % (2 * np.pi)


def saturate(value, min, max):
      if value >= max:
            return max
      elif value <= min:
            return min
      else:
            return value