import time
import threading
import cflib.crtp # type: ignore
from cflib.crazyflie.swarm import CachedCfFactory # type: ignore
from cflib.crazyflie.swarm import Swarm # type: ignore

class CrazyflieConnection:
    def __init__(self, uris):
        self.uris = uris
        self.swarm = None

    def update_uris(self, uris):
        self.uris = uris

    def connect(self):
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        self.swarm = Swarm(self.uris, factory=factory)
        self.swarm.parallel_safe(self.light_check)
        self.swarm.reset_estimators()

    def light_check(self, scf):
        scf.cf.param.set_value('led.bitmask', 255)
        time.sleep(2)
        scf.cf.param.set_value('led.bitmask', 0)
        time.sleep(2)

    def disconnect(self):
        self.swarm.close_links()