import time
import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import syncCrazyflie# type: ignore
from udp_client import UDP_Client
import traceback
import math 
import matplotlib.pyplot as plt
from vicon_connection_class import ViconInterface as vi
from timed_queue import TimedQueue
import cProfile
import pstats
import logging
from logging.handlers import RotatingFileHandler
from queue import Queue
import threading
from collections import namedtuple
import numpy as np
import pygame


# OBJECT NAMES
OBJECT_NAME = "CrazyfliePayload"
LEFT_DRONE_NAME = "PbodyCrazyFlie"
RIGHT_DRONE_NAME = "AtlasCrazyflie"

# CONSTANTS
YAW_THRESHOLD = 0.5
VELOCITY_THRESHOLD = 0.75
POSITION_THRESHOLD = 3.14
WEIGHTING = 0.5
beam_length = 0.4
ANGLE_GAIN = 0.125

uris = [
    'radio://0/81/2M/E6E7E6E7E6', #P-Body ON THE RIGHT
    'radio://0/80/2M/E7E7E7E7E7', #Atlas ON THE LEFT
]

reference = np.array([0, 0, 0])  # reference state
height = 0.8

#CONTROL QUEUE DEFINITIONS
Takeoff = namedtuple('Takeoff', ['height', 'time'])
Land = namedtuple('Land', ['time'])
Altitude = namedtuple('Goto', ['roll', 'pitch', 'yaw', 'altitude'])

# Reserved for the control loop, do not use in sequence
Quit = namedtuple('Quit', [])

# CONTROLLER SPECIFIC
Kr = matrix = np.array([
    [1.09544511501033, 1.36016126978046e-14, -9.75767735007051e-15],
    [2.75622589439528e-15, 1.09544511501029, -5.6636511851359e-14],
    [5.86112921741638e-15, -2.78272267634588e-14, 2.00000000000014]
])

Kx = np.array([
    [21.4793255440022, 64.522360771309, 1.91524565700855e-13, 4.0698479918769e-13, -3.95192434058514e-14, -5.38957167508448e-14],
    [7.75551252183115e-15, 5.55903059633902e-14, 24.1115729550083, 64.5253388025287, -4.74408815699575e-13, -7.84699588061605e-13],
    [-1.00601714715034e-13, -3.91355010105559e-13, -3.73652184809754e-13, -1.15421175892622e-12, 17.8525109479907, 29.6780345860254]
])

def control_thread():
    vicon = vi()
    vicon_thread = threading.Thread(target=vicon.main_loop)
    vicon_thread.start()
    print("starting vicon thread")

    logger = logging.getLogger('crazyflie_yaw_test')
    logger.setLevel(logging.INFO)

    # Set up a rotating file handler
    handler = RotatingFileHandler(
        'app.log',  # Log file name
        maxBytes=100000,  # Maximum size of a log file in bytes before rotation
        backupCount=3  # Number of backup files to keep
    )

    # Optional: Set a formatter for the log messages
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)

    # Add the handler to the logger
    logger.addHandler(handler)

    # Write an informational log message
    logging.info('Log started')

    # creates the udp client for plotting
    client = UDP_Client()

    roll = 0
    pitch = 0

    current_pos = np.array([0,0,0,0,0,0])
    prev_pos = np.array([0,0,0,0,0,0])

    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()
    
    try:
        time.sleep(0.1)
        offset = get_pos(vicon.getPos(OBJECT_NAME), current_pos)
        pbody_offset = get_pos(vicon.getPos("PbodyCrazyFlie"), current_pos)
        atlas_offset = get_pos(vicon.getPos("AtlasCrazyflie"), current_pos)

        print("Offset Pos & Rot: ",offset)
        
        waiting = True
        while(waiting):
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0: # A button
                        waiting = False
            time.sleep(0.25)
            print('waiting')
        
        print("Starting flight")
        logging.info('Starting flight')
            
        
        # takeoff 3 seconds
        controlQueues[0].put(Takeoff(height, 3))
        controlQueues[1].put(Takeoff(height, 3))
        
        time.sleep(3)
        
        flying = True
        start_time = time.time()
        current_pos = get_pos(vicon.getPos(OBJECT_NAME), current_pos) - offset
        prev_pos = current_pos
        current_time = time.time()
        prev_time = current_time
        y_tracker = 0
        prev_vel = np.array([0,0,0,0,0,0])
        print('starting altitude flight')

        while flying:
                
            time.sleep(0.01) #

            # safety timeout
            elapsed_time = time.time() - start_time
            if elapsed_time < 10: # time to stabilize
                reference = np.array([0, 0, 0])
            else:
                reference = np.array([np.sin((1/90) * elapsed_time), (-1/2)*np.sin((2/90) * elapsed_time), 0]) 

            if elapsed_time > 120:
                print("Timeout reached. Exiting loop.")
                break
            
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 1:  # B button
                        flying = False
                        logging.info('Flight disabled (B Button)')
                    if event.button == 3:  # Y button
                        logging.info('Emergency Stop (Y Button)')
                        raise Exception('Manual Emegency Stop') # emergency stop
            
            d_time = time.time() - current_time
            # update states
            current_pos = get_pos(vicon.getPos(OBJECT_NAME), current_pos) - offset
            
            current_vel = vicon.getVel(OBJECT_NAME)
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            x = np.array([current_pos[0],current_vel[0], current_pos[1],current_vel[1], current_pos[5],current_vel[5]])
            Cx = np.array([current_pos[0], current_pos[1], current_pos[5]])
            
            
            y_tracker += (Kr @ (reference - Cx))*d_time
            y_tracker = np.clip(y_tracker, -10, 10)
            #  Try without reference tracking first!
            stateReg = -Kx @ x
            u = stateReg + y_tracker
            
            yaw = current_pos[5]

            ## split the sum of forces into the the agents compomonents
            HT_thrusts = np.array([u[0], u[1]])
            moment_z = u[2]/(2*beam_length)
            
            rot_matrix = np.array([[np.cos(yaw), np.sin(yaw)],
                [-np.sin(yaw), np.cos(yaw)]])
            
            HT_thrusts = rot_matrix @ HT_thrusts
            bx_thrust = HT_thrusts[0]
            by_thrust = HT_thrusts[1]
            bx_1 = WEIGHTING * bx_thrust
            bx_2 = (1-WEIGHTING) * bx_thrust
            by_1 = 0.5 * by_thrust
            by_2 = 0.5 * by_thrust 
            
            roll_1 = 10*ANGLE_GAIN*bx_1
            roll_2 = 10*ANGLE_GAIN*bx_2
            
            pitch_1 = ANGLE_GAIN*(-by_1 - moment_z) # right one
            pitch_2 = ANGLE_GAIN*(-by_2 + moment_z) # left one
            
            # send to drones
            # Breaking down the control queue put operation into smaller parts
            yaw = math.degrees(current_pos[5])

            c_roll_1 = max(min(roll_1, 5), -5)
            c_roll_2 = max(min(roll_2, 5), -5)
            
            c_pitch_1 = max(min(pitch_1, 5), -5)
            c_pitch_2 = max(min(pitch_2, 5), -5)

            # Putting the Altitude command into the control queue
            controlQueues[0].put(Altitude(c_roll_1, c_pitch_1,  yaw, height))  # ATLAS RIGHT
            controlQueues[1].put(Altitude(c_roll_2, c_pitch_2, yaw, height))  # PBODY LEFT
            
            atlas = vicon.getPos("AtlasCrazyflie") - atlas_offset
            pbody = vicon.getPos("PbodyCrazyFlie") - pbody_offset
            
            atlasUF = vicon.getUF("AtlasCrazyflie") - atlas_offset
            pbodyUF = vicon.getUF("PbodyCrazyFlie") - pbody_offset

            client.send({
                OBJECT_NAME: {
                    "position": {"x": float(current_pos[0]),"y": float(current_pos[1]), "z": float(current_pos[2])},
                    "attitude": {"roll": math.degrees(float(current_pos[3])),"pitch": math.degrees(float(current_pos[4])),"yaw": math.degrees(float(current_pos[5]))},
                    "velocityT": {"x": float(current_vel[0]),"y": float(current_vel[1]), "z": float(current_vel[2])},
                    "velocityR": {"roll": math.degrees(float(current_vel[3])),"pitch": math.degrees(float(current_vel[4])),"yaw": math.degrees(float(current_vel[5]))},
                },
                "CONTROLLER":{
                    "wrench": {"x": float(u[0]), "y": float(u[1]), "moment z": float(u[2])},
                    "Rotated Thrusts": {"x" : float(HT_thrusts[0]), "y": float(HT_thrusts[1])},
                    "Queue Size": {"Atlas": controlQueues[0].qsize() , "P-Body":controlQueues[1].qsize() },
                    "controller Components": {"Reference Tracking x": float(y_tracker[0]),"Reference Tracking y": float(y_tracker[1]),"Reference Tracking z": float(y_tracker[2]), "State Regulation x": float(stateReg[0]),"State Regulation y": float(stateReg[1]),"State Regulation z": float(stateReg[2])},  
                },
                "ATLAS":{
                    "setpoint": {"clamped roll": float(c_roll_1),"clamped pitch": float(c_pitch_1),"yaw": float(yaw)},
                    "position": {"x": float(atlas[0]),"y": float(atlas[1]), "z": float(atlas[2])},
                    "attitude": {"roll": math.degrees(float(atlas[3])),"pitch": math.degrees(float(atlas[4])),"yaw": math.degrees(float(atlas[5]))},
                    "unfiltered": {"x": float(atlasUF[0]),"y": float(atlasUF[1]), "z": float(atlasUF[2]), "roll": math.degrees(float(atlasUF[3])),"pitch": math.degrees(float(atlasUF[4])),"yaw": math.degrees(float(atlasUF[5]))}
                },
                "P-BODY":{
                    "setpoint": {"clamped roll": float(c_roll_2),"clamped pitch": float(c_pitch_2),"yaw": float(yaw)},
                    "position": {"x": float(pbody[0]),"y": float(pbody[1]), "z": float(pbody[2])},
                    "attitude": {"roll": math.degrees(float(pbody[3])),"pitch": math.degrees(float(pbody[4])),"yaw": math.degrees(float(pbody[5]))},
                    "unfiltered": {"x": float(pbodyUF[0]),"y": float(pbodyUF[1]), "z": float(pbodyUF[2]), "roll": math.degrees(float(pbodyUF[3])),"pitch": math.degrees(float(pbodyUF[4])),"yaw": math.degrees(float(pbodyUF[5]))}
                },
            })
        
        controlQueues[0].put(Land(3))
        controlQueues[1].put(Land(3))

        # controlQueues[1].put(Land(3))
    except Exception as e:
        for ctrl in controlQueues:
            ctrl.put(Quit())
        print(traceback.format_exc())
        logging.exception(traceback.format_exc())
        print(e)
    finally:
        vicon.end()
        client.close()

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
            scf.cf.commander.send_custom_altitude_setpoint(command.roll, command.pitch ,command.yaw, command.altitude)
        else:
            scf.cf.high_level_commander.stop()
            time.sleep(0.1)

            print('Warning! unknown command {} for uri {}'.format(command,cf.uri))

def light_check(scf):
    def activate_led_bit_mask(scf):
        scf.cf.param.set_value('led.bitmask', 255)

    def deactivate_led_bit_mask(scf):
        scf.cf.param.set_value('led.bitmask', 0)
    
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)

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
    # controlQueues = [TimedQueue(0.1) for _ in range(len(uris))]
    controlQueues = [Queue() for _ in range(len(uris))]

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    
    with Swarm(uris, factory=factory) as swarm:
        try:    
            swarm.parallel_safe(set_params)
            swarm.parallel_safe(light_check)
            print('Light check done')
            swarm.reset_estimators()
            print('Estimators reset')
            
            threading.Thread(target=control_thread).start()
            print('Control thread started')
            swarm.parallel_safe(update_crazy_controller)
            print('Crazyflie threads started')
        except Exception as e:
            swarm.parallel_safe(stop)
            time.sleep(0.1)

            print(traceback.format_exc())
        
    pygame.quit()
    # Convert results to numpy arrays for easier handling
        
