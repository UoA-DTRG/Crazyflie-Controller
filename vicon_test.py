import pyvicon_datastream as pv

VICON_TRACKER_IP = "10.0.108.3"
OBJECT_NAME = "My_Object"

vicon_client = pv.PyViconDatastream()
ret = vicon_client.connect(VICON_TRACKER_IP)

if ret != pv.Result.Success:
    print(f"Connection to {VICON_TRACKER_IP} failed")
else:
    print(f"Connection to {VICON_TRACKER_IP} successful")








# from pyvicon_datastream import tools

# VICON_TRACKER_IP = "10.0.108.3"
# OBJECT_NAME = "My_Object"

# mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
# while(True):
#     position = mytracker.get_position(OBJECT_NAME)
#     print(f"Position: {position}")
#     time.sleep(0.5)