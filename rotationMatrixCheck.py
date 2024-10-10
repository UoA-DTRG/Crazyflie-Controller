
import math
import numpy as np


rot_yaw = math.radians(90)
HT_thrusts = np.array([1, 1])


rot_matrix = np.array([[np.cos(rot_yaw), np.sin(rot_yaw)],
    [-np.sin(rot_yaw), np.cos(rot_yaw)]])

HT_thrusts = rot_matrix @ HT_thrusts

print(HT_thrusts)