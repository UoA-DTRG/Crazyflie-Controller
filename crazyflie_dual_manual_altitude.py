import time

import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore
from cflib.crazyflie import syncCrazyflie# type: ignore
import traceback

# import control# type: ignore
import numpy as np

import pygame

uris = [
    'radio://0/80/2M/E7E7E7E7E7', #Atlas
    'radio://0/81/2M/E6E7E6E7E6', #P-Body
]

REF = np.array([0, 0, 0])  # reference state
height = 1.0
# LQR CONTROLLER SETUP


def light_check(scf):
    def activate_led_bit_mask(scf):
        scf.cf.param.set_value('led.bitmask', 255)

    def deactivate_led_bit_mask(scf):
        scf.cf.param.set_value('led.bitmask', 0)
    
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)
 


# def update_controller(scf,u):
#     commander = scf.cf.high_level_commander
#     # calculate the control input based on the state x and the reference REF

#     roll = u[0]
#     pitch = u[1]
#     yawrate = u[2]

#     commander.send_zdistance_setpoint(self, roll, pitch, yawrate, height)
#     time.sleep(0.1)




def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3)
    
def land(scf):
    commander= scf.cf.high_level_commander

    commander.land(0.0, 5.0)
    time.sleep(5)

    commander.stop()  

# EMERGENCY STOP
def stop(scf):
    commander= scf.cf.high_level_commander
    commander.stop()
    print('EMERGENCY STOP OCCURED')
    


if __name__ == '__main__':
    # K, S, E = calculateControllerGains()
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
            
            while flying:
                current_time = time.time()
                elapsed_time = current_time - start_time

                if elapsed_time > 10:
                    print("Timeout reached. Exiting loop.")
                    break

                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION:
                        roll = controller.get_axis(3) * 5
                        pitch = controller.get_axis(4) * 5
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == 1:  # B button
                            flying = False
                        if event.button == 3:  # Y button
                            raise Exception('Manual Emegency Stop') # emergency stop

                u = np.array([roll, pitch, yawrate])
                print("Control input: ", u)
                time.sleep(0.02) # 50hz
                # update_controller(scf, u)

            swarm.parallel_safe(land)
        except Exception as e:
            swarm.parallel(stop)
            print(traceback.format_exc())

    pygame.quit()