import pyvicon_datastream as pv
from pyvicon_datastream import tools
import asyncio
from PyQt5.QtCore import QObject, pyqtSignal
from multiprocessing import Process, Queue, TimeoutError as MPTimeoutError

class ViconConnection(QObject):
    position_updated = pyqtSignal(float, float, float)  # Define the signal, emitting a tuple for position

    def __init__(self, vicon_tracker_ip, object_name):
        super().__init__()  # Properly initialize QObject
        self.vicon_tracker_ip = vicon_tracker_ip
        self.object_name = object_name
        self.vicon_client = pv.PyViconDatastream()
        self.mytracker = None
        self.connected = False

    def _attempt_connection(self, queue):
        try:
            result = self.vicon_client.connect(self.vicon_tracker_ip)
            queue.put(result)
        except Exception as e:
            queue.put(e)

    def connect(self):
        timeout = 5  # Timeout in seconds
        queue = Queue()
        process = Process(target=self._attempt_connection, args=(queue,))
        process.start()
        
        try:
            result = queue.get(timeout=timeout)
            if isinstance(result, Exception):
                raise result
        except MPTimeoutError:
            print(f"Connection to {self.vicon_tracker_ip} timed out")
            process.terminate()  # Forcefully terminate the process
            process.join()  # Ensure the process has finished
            self.connected = False
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            process.terminate()  # Ensure the process is terminated
            process.join()  # Ensure the process has finished
            self.connected = False
            return False
        
        process.join()  # Ensure the process has finished

        if result != pv.Result.Success:
            print(f"Connection to {self.vicon_tracker_ip} failed")
            self.connected = False
            return False
        else:
            print(f"Connection to {self.vicon_tracker_ip} successful")
            self.mytracker = tools.ObjectTracker(self.vicon_tracker_ip)
            self.connected = True
            return True
            
    def start_tracking(self):
        if not self.connected:
            print("Not connected to the Vicon Tracker. Please connect first.")
            return
        vicon = ViconConnection(self.vicon_tracker_ip, self.object_name)
        asyncio.run(vicon.get_position())

    async def get_position(self):
        print("Vicon Connection Open")
        while self.connected:
            position = self.mytracker.get_position(self.object_name)
            self.position_updated.emit(position)
            await asyncio.sleep(0.01)  # 100Hz to match the vicon stream
        print("Vicon Connection Closed")