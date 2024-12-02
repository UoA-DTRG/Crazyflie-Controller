import time

import numpy as np
import math 
from vicon_connection_class import ViconInterface as vi

OBJECT_NAME = "AtlasCrazyflie"


try:
    vicon = vi()
    vicon_thread = Thread(target=vicon.main_loop)

    vicon_thread.start()

    while True:

        print(vicon.getLatestNED(OBJECT_NAME))
        time.sleep(0.05)

except KeyboardInterrupt:
    pass
except Exception as e:
    print(e)
