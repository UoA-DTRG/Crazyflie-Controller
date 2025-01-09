import time
import numpy as np
import math
from vicon_connection_class import ViconInterface as vi
from threading import Thread
from datetime import datetime as dt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from udp_client import UDP_Client

vicon = vi()
try:
    # Create the Vicon connection and start its thread
    vicon_thread = Thread(target=vicon.main_loop)
    vicon_thread.start()
    # creates the udp client for plotting
    client = UDP_Client()
    print("running")
    while True:
        atlas = vicon.getPos("AtlasCrazyflie")
        # print(f'atlas = {atlas}')
        pbody = vicon.getPos("PbodyCrazyFlie")
        # print(f'pbody = {pbody}')
        current_pos = vicon.getPos("CrazyfliePayload")
        pbodyV = vicon.getVel("PbodyCrazyFlie")
        # print(f'payload = {current_pos}')
        if atlas and pbody and current_pos:
            client.send({
                    "Payload": {
                        "position": {"x": float(current_pos[0]),"y": float(current_pos[1]), "z": float(current_pos[2])},
                        "attitude": {"roll": math.degrees(float(current_pos[3])),"pitch": math.degrees(float(current_pos[4])),"yaw": math.degrees(float(current_pos[5]))},
                    },
                    "ATLAS":{
                        "position": {"x": float(atlas[0]),"y": float(atlas[1]), "z": float(atlas[2])},
                        "attitude": {"roll": math.degrees(float(atlas[3])),"pitch": math.degrees(float(atlas[4])),"yaw": math.degrees(float(atlas[5]))},
                    },
                    "P-BODY":{
                        "position": {"x": float(pbody[0]),"y": float(pbody[1]), "z": float(pbody[2])},
                        "velocity": {"x": float(pbodyV[0]),"y": float(pbodyV[1]), "z": float(pbodyV[2])},
                        "attitude": {"roll": math.degrees(float(pbody[3])),"pitch": math.degrees(float(pbody[4])),"yaw": math.degrees(float(pbody[5]))},
                        "attitude rate": {"roll": math.degrees(float(pbodyV[3])),"pitch": math.degrees(float(pbodyV[4])),"yaw": math.degrees(float(pbodyV[5]))}
                    },
                })
            # print("------------------------------------------- SUCCESS -------------------------------------------")
        # else:
            # print("No data")
            # print("atlas")
            # print(atlas)
            # print("pbody")
            # print(pbody)
            # print("current_pos")
            # print(current_pos)
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
except Exception as e:
    print(e)
finally:
    vicon.end()

