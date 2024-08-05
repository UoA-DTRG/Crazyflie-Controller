import pyvicon_datastream as pv
from pyvicon_datastream import tools
import asyncio

class ViconConnection:
    def __init__(self, vicon_tracker_ip, object_name):
        self.vicon_tracker_ip = vicon_tracker_ip
        self.object_name = object_name
        self.vicon_client = pv.PyViconDatastream()
        self.mytracker = tools.ObjectTracker(self.vicon_tracker_ip)
        self.connected = False
    
    def connect(self):
        ret = self.vicon_client.connect(self.vicon_tracker_ip)
        if ret != pv.Result.Success:
            print(f"Connection to {self.vicon_tracker_ip} failed")
            self.connected = False
            return False
        else:
            print(f"Connection to {self.vicon_tracker_ip} successful")
            self.connected = True
            return True

    def start_tracking(self):
        if not self.connected:
            print("Not connected to the Vicon Tracker. Please connect first.")
            return
        vicon = ViconConnection(self.vicon_tracker_ip, self.object_name)
        asyncio.run(vicon.get_position())

    async def get_position(self):
        while True:
            position = self.mytracker.get_position(self.object_name)
            await asyncio.sleep(0.01)  # 100Hz to match the vicon stream