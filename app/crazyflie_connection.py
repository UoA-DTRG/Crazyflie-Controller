import time
import threading
import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore

class CrazyflieConnection:
    def __init__(self, uris):
        self.uris = uris

    def light_check(self, scf):
        scf.cf.param.set_value('led.bitmask', 255)
        time.sleep(2)
        scf.cf.param.set_value('led.bitmask', 0)
        time.sleep(2)

    def take_off(self, scf):
        commander= scf.cf.high_level_commander

        commander.takeoff(1.0, 2.0)
        time.sleep(3)

    def land(self, scf):
        commander= scf.cf.high_level_commander

        commander.land(0.0, 2.0)
        time.sleep(2)

        commander.stop()
    
    def run(self):
        def run_swarm():
            cflib.crtp.init_drivers()
            factory = CachedCfFactory(rw_cache='./cache')
            with Swarm(self.uris, factory=factory) as swarm:
                swarm.parallel_safe(self.light_check)
                swarm.reset_estimators()
        thread = threading.Thread(target=run_swarm)
        thread.start()