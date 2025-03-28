import logging
import sys
import time
from threading import Event

import cflib.crtp# type: ignore
from cflib.crazyflie import Crazyflie# type: ignore
from cflib.crazyflie.log import LogConfig# type: ignore
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie# type: ignore
from cflib.positioning.motion_commander import MotionCommander# type: ignore
from cflib.utils import uri_helper# type: ignore


URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(15)
        mc.stop()

if __name__ == '__main__':

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                cb=param_deck_flow)
        time.sleep(1)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        take_off_simple(scf)