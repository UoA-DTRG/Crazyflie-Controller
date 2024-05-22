import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie import syncCrazyflie

uris = [
    'radio://0/80/2M/E7E7E7E7E7', #Atlas
    'radio://0/81/2M/E6E7E6E7E6', #P-Body
]

def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)


def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)

def land(scf):
    commander= scf.cf.high_level_commander

    commander.land(0.0, 5.0)
    time.sleep(2)

    commander.stop()
# The layout of the positions (1m apart from each other):
#   <------ 1 m ----->
#   0                1
#          ^              ^
#          |Y             |
#          |              |
#          +------> X    1 m
#                         |
#                         |
#   3               2     .


h = 0.0 # Height increase between sequence points
x1, y1 = +1.0, +1.0
x0, y0 = -1.0, -1.0

#    x   y   z  time
sequence0 = [
    (x0+1, y1, h, 3.0), #top left
    (x1+1, y1, h, 3.0), #top Right
    (x1+1, y0, h, 3.0), #Bottom right
    (x0+1, y0, h, 3.0), #bottom left
    (x0+1,  0, h, 3.0), #left middle
]

sequence1 = [
    (x1-1, y0, h, 3.0), #Bottom right
    (x0-1, y0, h, 3.0), #bottom left
    (x0-1, y1, h, 3.0), #top left
    (x1-1, y1, h, 3.0), #top Right
    (x1-1,  0, h, 3.0), #left middle
]

seq_args = {
    uris[0]: [sequence0],
    uris[1]: [sequence1],
}

def run_sequence(scf: syncCrazyflie.SyncCrazyflie, sequence):
    cf = scf.cf

    for arguments in sequence:
        commander = scf.cf.high_level_commander

        x, y, z = arguments[0], arguments[1], arguments[2]
        duration = arguments[3]

        print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
        commander.go_to(x, y, z, 0, duration, relative=True)
        time.sleep(duration)

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(light_check)
        swarm.reset_estimators()
        
        swarm.parallel_safe(run_sequence, args_dict=seq_args)