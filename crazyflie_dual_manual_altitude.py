import time

import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore
from cflib.crazyflie import syncCrazyflie# type: ignore

import control# type: ignore
import numpy as np

import pygame

uris = [
    'radio://0/80/2M/E7E7E7E7E7', #Atlas
    'radio://0/81/2M/E6E7E6E7E6', #P-Body
]

REF = np.array([0, 0, 0])  # reference state
height = 1.0
# LQR CONTROLLER SETUP

def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(height, 3.0)
    time.sleep(3)

def land(scf):
    commander= scf.cf.high_level_commander

    commander.land(0.0, 5.0)
    time.sleep(2)

    commander.stop()

def update_controller(scf,u):
    commander = scf.cf.high_level_commander
    # calculate the control input based on the state x and the reference REF

    roll = u[0]
    pitch = u[1]
    yawrate = u[2]

    commander.send_zdistance_setpoint(self, roll, pitch, yawrate, height)
    time.sleep(0.1)



def run_controller():
    with syncCrazyflie('radio://0/80/2M/E7E7E7E7E7', cf=Crazyflie(rw_cache='./cache')) as scf:
        waiting = True
        while(waiting):
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0: # A button
                        waiting = False
        print("Starting flight")
        take_off(scf)
        time.sleep(5)
        roll = 0
        pitch = 0
        yawrate = 0
        flying = True
        while(flying == True):
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    roll = controller.get_axis(3)*5
                    pitch = controller.get_axis(4)*5
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 1:  # B button
                        flying = False
                    if event.button == 3: # Y button
                        scf.stop() # emergency stop
                
            u = np.array([roll, pitch, yawrate])
            print("Control input: ", u)

            update_controller(scf,u)


        land(scf)


if __name__ == '__main__':
    K, S, E = calculateControllerGains()
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()

    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(light_check)
        swarm.reset_estimators()
        swarm.parallel_safe(run_controller, args_dict=seq_args)

    pygame.quit()