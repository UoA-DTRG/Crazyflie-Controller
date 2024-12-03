import time

import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import syncCrazyflie# type: ignore
import traceback
import math 
import matplotlib.pyplot as plt
from vicon_connection_class import ViconInterface as vi

import cProfile
import pstats

from queue import Queue
import threading
from collections import namedtuple
import numpy as np

import pygame

OBJECT_NAME = "mug"

LEFT_DRONE_NAME = "PbodyCrazyFlie"
RIGHT_DRONE_NAME = "AtlasCrazyflie"

YAW_THRESHOLD = 0.5
VELOCITY_THRESHOLD = 0.75
POSITION_THRESHOLD = 3.14
WEIGHTING = 0.5
beam_length = 0.4

uris = [
    'radio://0/80/2M/E7E7E7E7E7', #Atlas ON THE RIGHT
]

reference = np.array([0, 0, 0])  # reference state
height = 0.6

# Possible commands, all times are in seconds
Takeoff = namedtuple('Takeoff', ['height', 'time'])
Land = namedtuple('Land', ['time'])
Altitude = namedtuple('Goto', ['roll', 'pitch', 'yawrate', 'altitude'])
# Reserved for the control loop, do not use in sequence
Quit = namedtuple('Quit', [])

cap_limit = 0.12217305

x_history = []
u_history = []  
ref_history = []
full_history = []
yaw_history = []
yaw_track_history = []
setpoint_history = []
timeT = []

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
 
def control_thread():
   
    vicon = vi()
    vicon_thread = threading.Thread(target=vicon.main_loop)
    vicon_thread.start()
    
    
    
    roll = 0
    pitch = 0

    current_pos = np.array([0,0,0,0,0,0])
    prev_pos = np.array([0,0,0,0,0,0])

    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()
    
    try:
        time.sleep(0.5)
        offset = get_pos(vicon.getPos(OBJECT_NAME), current_pos)
        
        # left_drone_offset = np.array([0,0,0,0,0,0])
        # left_drone_offset = get_pos(mytracker.get_position(LEFT_DRONE_NAME), left_drone_offset)
        # right_drone_offset = np.array([0,0,0,0,0,0])
        # right_drone_offset = get_pos(mytracker.get_position(RIGHT_DRONE_NAME), right_drone_offset)
        # left_drone_pos = get_pos(mytracker.get_position(LEFT_DRONE_NAME), left_drone_offset) - left_drone_offset
        # right_drone_pos = get_pos(mytracker.get_position(RIGHT_DRONE_NAME), right_drone_offset) - right_drone_offset
        
        
        print("Offset Pos & Rot: ",offset)
        
        waiting = True
        while(waiting):
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0: # A button
                        waiting = False
            time.sleep(0.5)
            print('waiting')
        
        print("Starting flight")
        
     
        
        
        # takeoff 3 seconds
        controlQueues[0].put(Takeoff( height, 3))
        # controlQueues[1].put(Takeoff( height, 3))
        time.sleep(3)
        
        flying = True
        start_time = time.time()
        
        current_pos = get_pos(vicon.getPos(OBJECT_NAME), current_pos) - offset
        prev_pos = current_pos
    
        current_time = time.time()
        prev_time = current_time
   
        
        
        y_tracker = 0

        prev_vel = np.array([0, 0, 0,0,0,0])
        print('starting altitude flight')


        while flying:
                
            # safety timeout
            elapsed_time = time.time() - start_time

            if elapsed_time > 20:
                print("Timeout reached. Exiting loop.")
                break
            
            for event in pygame.event.get():
                # if event.type == pygame.JOYAXISMOTION:
                    # roll = controller.get_axis(3) * 5
                    # pitch = controller.get_axis(4) * 5
                    # if abs(roll) < 1:
                    #     roll = 0
                    # if abs(pitch) < 1:
                    #     pitch = 0
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 1:  # B button
                        flying = False
                    if event.button == 3:  # Y button
                        raise Exception('Manual Emegency Stop') # emergency stop
            
        
            d_time = time.time() - current_time
            # update states
            current_pos = get_pos(vicon.getPos(OBJECT_NAME), current_pos) - offset
                       
            current_vel = vicon.getVel(OBJECT_NAME)
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # #yaw P control (hopefully works well enough)
            # left_drone_pos = get_pos(mytracker.get_position(LEFT_DRONE_NAME), left_drone_pos) - left_drone_offset
            # right_drone_pos = get_pos(mytracker.get_position(RIGHT_DRONE_NAME), right_drone_pos) - right_drone_offset
            # yawrate1 = 1.4 * (x[4] - left_drone_pos[5])
            # yawrate2 = 1.4 * (x[4] - right_drone_pos[5])
            # yaw_track_history.append([math.degrees(left_drone_pos[5]),math.degrees(right_drone_pos[5]), math.degrees(x[4])])
            # yaw_history.append([math.degrees(yawrate1), math.degrees(yawrate2)])
               
            # send to drones
            controlQueues[0].put(Altitude(max(min(math.degrees(current_pos[3])/10,15),-15), max(min(math.degrees(current_pos[4])/10,15),-15), math.degrees(current_pos[5]), height)) #ATLAS RIGHT
            # controlQueues[1].put(Altitude(math.degrees(0), math.degrees(0), math.degrees(yaw), height)) #PBODY LEFT
            
            # print(d_time)
            time.sleep(0.0011) # just under 100hz
        
        controlQueues[0].put(Land(3))
        # controlQueues[1].put(Land(3))
    except Exception as e:
        for ctrl in controlQueues:
            ctrl.put(Quit())
        print(traceback.format_exc())
    finally:
        vicon.end()
    
    

def update_crazy_controller(scf):
    pr = cProfile.Profile()
    pr.enable()
    
    control = controlQueues[uris.index(scf.cf.link_uri)]
    while True:
        command = control.get()
        if type(command) is Quit:
            scf.cf.high_level_commander.stop()
            time.sleep(0.1)
            pr.disable()
            stats = pstats.Stats(pr)
            stats.sort_stats(pstats.SortKey.CUMULATIVE)
            pr.dump_stats('output.prof')
            stream = open('output.txt', 'w')
            stats = pstats.Stats('output.prof', stream=stream)
            stats.sort_stats(pstats.SortKey.CUMULATIVE)
            stats.print_stats()
            return
        elif type(command) is Takeoff:
            scf.cf.high_level_commander.takeoff(command.height, command.time)
        elif type(command) is Land:
            scf.cf.high_level_commander.land(0.0, command.time)
            time.sleep(0.1)
        elif type(command) is Altitude:
            scf.cf.commander.send_position_setpoint(0.0, 0.0 ,command.altitude, command.yawrate)
        else:
            scf.cf.high_level_commander.stop()
            time.sleep(0.1)

            print('Warning! unknown command {} for uri {}'.format(command,
                                                                  cf.uri))
    

def take_off(scf):
    commander= scf.cf.high_level_commander

    commander.takeoff(height, 2.0)
    time.sleep(3)
    
def land(scf):
    scf.cf.commander.send_notify_setpoint_stop()
    scf.cf.high_level_commander.land(0.0, 5.0)
    time.sleep(5)

    scf.cf.high_level_commander.stop()
    time.sleep(0.1)

def param_stab_est_callback(name, value):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)

def set_params(scf):
    groupstr = "flightmode"
    namestr = "stabModeYaw"
    full_name = groupstr + '.' + namestr
    # scf.param.add_update_callback(group=groupstr, name=namestr,
    #                              cb=param_stab_est_callback)
    scf.cf.param.set_value(full_name, 1)

    namestr = "althold"
    full_name = groupstr + '.' + namestr
    # scf.param.add_update_callback(group=groupstr, name=namestr,
    #                              cb=param_stab_est_callback)
    scf.cf.param.set_value(full_name, 1)
    

# EMERGENCY STOP
def stop(scf):
    commander= scf.cf.high_level_commander
    commander.stop()
    time.sleep(0.1)

    print('EMERGENCY STOP OCCURED')
    
def hold_pos(scf):
    commander= scf.cf.high_level_commander
    commander.go_to(0, 0, 0, 0, 1.0, relative=True)
    time.sleep(2)

def get_pos(position, current_pos):
    if position is not None:
        return np.array(position)
    else:
        return current_pos
        print('LOST SIGHT OF OBJECT')

if __name__ == '__main__':
    controlQueues = [Queue() for _ in range(len(uris))]
        
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    
    with Swarm(uris, factory=factory) as swarm:
        try:    
            swarm.parallel_safe(set_params)
            swarm.parallel_safe(light_check)
            swarm.reset_estimators()
            
            threading.Thread(target=control_thread).start()

            swarm.parallel_safe(update_crazy_controller)
            
        except Exception as e:
            swarm.parallel_safe(stop)
            time.sleep(0.1)

            print(traceback.format_exc())
        
    pygame.quit()
    # Convert results to numpy arrays for easier handling
        
