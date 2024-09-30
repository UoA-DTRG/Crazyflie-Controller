import time
import pyvicon_datastream as pv
from pyvicon_datastream import tools
import numpy as np
import math 

VICON_TRACKER_IP = "192.168.10.1"
OBJECT_NAME = "mug"

vicon_client = pv.PyViconDatastream()
ret = vicon_client.connect(VICON_TRACKER_IP)
mytracker = tools.ObjectTracker(VICON_TRACKER_IP)

if ret != pv.Result.Success:
    print(f"Connection to {VICON_TRACKER_IP} failed")
else:
    print(f"Connection to {VICON_TRACKER_IP} successful")



def get_pos(position, current_pos):
    if len(position[2]) > 0:
        obj_data = position[2][0]
        x, z, y = obj_data[2], obj_data[3], obj_data[4]
        x_rot, z_rot, y_rot  = math.degrees(obj_data[5]), math.degrees(obj_data[6]), math.degrees(obj_data[7])
        return np.array([x/1000, y/1000, z/1000, x_rot, y_rot, z_rot])
    else:
        return current_pos

current_pos = np.array([0,0,0,0,0,0])
prev_pos = np.array([0,0,0,0,0,0])

current_pos = get_pos(mytracker.get_position(OBJECT_NAME), current_pos)
prev_pos = current_pos
mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
current_time = time.time()
prev_time = current_time

while(True):
    d_time = time.time() - current_time
    current_pos = get_pos(mytracker.get_position(OBJECT_NAME), current_pos)
    current_vel = (np.subtract(current_pos,prev_pos)) / d_time
    print("POS:" ,current_pos , "VEL:", current_vel)
    prev_pos = current_pos
    current_time = time.time()
    time.sleep(0.5)
