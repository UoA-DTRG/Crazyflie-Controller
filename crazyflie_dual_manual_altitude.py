import time

import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import syncCrazyflie# type: ignore
import traceback
import pyvicon_datastream as pv
from pyvicon_datastream import tools

VICON_TRACKER_IP = "192.168.10.1"
OBJECT_NAME = "mug"

import numpy as np

import pygame

uris = [
    'radio://0/80/2M/E7E7E7E7E7', #Atlas
    'radio://0/81/2M/E6E7E6E7E6', #P-Body
]

REF = np.array([0, 0, 0])  # reference state
height = 0.6


yaw1_offset = 0
yaw2_offset = 0


def light_check(scf):
    def activate_led_bit_mask(scf):
        scf.cf.param.set_value('led.bitmask', 255)

    def deactivate_led_bit_mask(scf):
        scf.cf.param.set_value('led.bitmask', 0)
    
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)
 


def update_controller(scf,roll,pitch,yawrate,altitude):
    scf.cf.commander.send_zdistance_setpoint(roll, pitch, yawrate, altitude)


def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(height, 2.0)
    time.sleep(3)
    
def land(scf):
    scf.cf.commander.send_notify_setpoint_stop()
    scf.cf.high_level_commander.land(0.0, 5.0)
    time.sleep(5)

    scf.cf.high_level_commander.commander.stop()  

# EMERGENCY STOP
def stop(scf):
    commander= scf.cf.high_level_commander
    commander.stop()
    print('EMERGENCY STOP OCCURED')
    
def hold_pos(scf):
    commander= scf.cf.high_level_commander
    commander.go_to(0, 0, 0, 0, 1.0, relative=True)
    time.sleep(2)


if __name__ == '__main__':
    
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()
    
    roll = 0
    pitch = 0
    yawrate = 0

    with Swarm(uris, factory=factory) as swarm:
        try:

            swarm.parallel_safe(light_check)
            swarm.reset_estimators()            
            
            waiting = True
            while(waiting):
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == 0: # A button
                            waiting = False
                time.sleep(0.5)
                print('waiting')
            
            print("Starting flight")
            
            swarm.parallel_safe(take_off)
            
            flying = True
            start_time = time.time()
            
            
            print('starting altitude flight')
            while flying:
                # safety timeout
                elapsed_time = time.time() - start_time
                if elapsed_time > 20:
                    print("Timeout reached. Exiting loop.")
                    break
                
                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION:
                        roll = controller.get_axis(3) * 5
                        pitch = controller.get_axis(4) * 5
                        if abs(roll) < 1:
                            roll = 0
                        if abs(pitch) < 1:
                            pitch = 0
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == 1:  # B button
                            flying = False
                        if event.button == 3:  # Y button
                            raise Exception('Manual Emegency Stop') # emergency stop
                print("Control input: ", [roll, pitch, yawrate, height])
                args_dict = {
                    uris[0]: [roll, pitch, yawrate, height],
                    uris[1]: [roll, pitch, yawrate, height],
                }
                swarm.parallel_safe(update_controller, args_dict = args_dict)
              
                time.sleep(0.001) # 100hz
                
            swarm.parallel_safe(land)
        except Exception as e:
            swarm.parallel(stop)
            print(traceback.format_exc())

    pygame.quit()