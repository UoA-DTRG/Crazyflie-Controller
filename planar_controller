import time

import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore
from cflib.crazyflie import syncCrazyflie# type: ignore

uris = [
    'radio://0/80/2M/E7E7E7E7E7', #Atlas
    'radio://0/81/2M/E6E7E6E7E6', #P-Body
]

def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)

def land(scf):
    commander= scf.cf.high_level_commander

    commander.land(0.0, 5.0)
    time.sleep(2)

    commander.stop()

def run_controller():
    with syncCrazyflie('radio://0/80/2M/E7E7E7E7E7', cf=Crazyflie(rw_cache='./cache')) as scf:
        take_off(scf)
        time.sleep(5)
        land(scf)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(light_check)
        swarm.reset_estimators()
        
        swarm.parallel_safe(run_controller, args_dict=seq_args)