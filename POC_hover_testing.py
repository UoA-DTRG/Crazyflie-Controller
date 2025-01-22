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
# Kr = matrix = np.array([
#     [6.32455532033701, -1.92611708090559e-14, -2.51364697870046e-15],
#     [-5.11540256539333e-14, 5.99999999999965, 1.87222361954461e-14],
#     [-4.75035904976474e-15, -1.2170348703185e-12, 1.99999999999997]
# ])

# Kx = np.array([
#     [26.6274133227985, 43.4037667564728, -7.76461230836858e-14, -1.52606537403102e-13, -3.10094451126148e-14, -7.2568445419908e-14],
#     [1.9122942608089e-14, 7.80944765291056e-14, 26.9778082342948, 45.6501707938696, 9.5871237349572e-14, 1.54384453940806e-13],
#     [-2.2128302669098e-13, -3.22789743035866e-13, -5.46392521542712e-12, -9.16730358643293e-12, 13.563245018237, 20.9904016551837]
# ])

# test 004
# Kr = matrix = np.array([
#     [0.774596669241478, -7.57869734094658e-15, 2.48569466688542e-15],
#     [1.43229129293392e-16, 0.774596669241466, 1.74517030662685e-15],
#     [-1.82962367597883e-14, 3.50430342890093e-14, 1.4142135623731]
# ])

# Kx = np.array([
#     [14.8203309199592, 38.4988727916904, -1.17193125186401e-13, -2.48986395355086e-13, 1.98705962319181e-14, 2.73892732865911e-14],
#     [1.07662524994272e-14, 2.78180472623526e-14, 16.7226731259642, 38.5024797661155, 1.75468728301679e-14, 2.21988550915834e-14],
#     [-2.75293494341881e-13, -6.07006840732273e-13, 1.02641173125862e-12, 2.55456252800242e-12, 12.3978342486397, 18.98804015896]
# ])

#pretty damn good - test 005
# Kr = matrix = np.array([
#     [0.774596669241478, -7.57869734094658e-15, 2.48569466688542e-15],
#     [1.43229129293392e-16, 0.774596669241466, 1.74517030662685e-15],
#     [-1.82962367597883e-14, 3.50430342890093e-14, 1.4142135623731]
# ])

# Kx = np.array([
#     [14.8203309199592, 38.4988727916904, -1.17193125186401e-13, -2.48986395355086e-13, 1.98705962319181e-14, 2.73892732865911e-14],
#     [1.07662524994272e-14, 2.78180472623526e-14, 16.7226731259642, 38.5024797661155, 1.75468728301679e-14, 2.21988550915834e-14],
#     [-2.75293494341881e-13, -6.07006840732273e-13, 1.02641173125862e-12, 2.55456252800242e-12, 12.3978342486397, 18.98804015896]
# ])

# test 006 - higher reference tracking and lower cost of x,y usage (R[0] R[1] reduced)
Kr = matrix = np.array([
    [4.47213595500079, -1.8967683320921e-14, -3.50456964462182e-14],
    [2.6293003263237e-13, 4.47213595499975, 9.06520416523829e-13],
    [1.04110497249628e-13, 2.36015136098926e-13, 2.4494897427831]
])

Kx = np.array([
    [51.8497433567121, 121.686352093861, 1.81748043063742e-13, 5.53418333850863e-13, -1.41356425288108e-13, -1.92214048719555e-13],
    [3.05610238083765e-12, 7.05058795093076e-12, 57.3447937525281, 121.689648545881, 5.14071864671325e-12, 7.02550213697767e-12],
    [1.15947326189431e-12, 2.64395874483217e-12, 2.90176902216505e-12, 6.24696614581105e-12, 13.8935425005884, 18.9897738308771]
])

# eg. figure_eight(2, 1, 45 ...) = 2m width, 1m height, 45s total time
def figure_eight(width, height, total_time, time, time_offset = 0, horizontal = True):
    if horizontal:
        width_freq_modifier = 1
        height_freq_modifier = 2
    else:
        width_freq_modifier = 2
        height_freq_modifier = 1
    reference = np.array([0.5 * width * np.sin((2 * np.pi * width_freq_modifier/total_time) * (time - time_offset)), - 0.5 * height * np.sin((2 * np.pi * height_freq_modifier/total_time) * (time - time_offset)), 0]) 
    return reference

def step_offset(elapsed_time, step_time, start_x=0, start_y=0, start_yaw=0, x=0, y=0,yaw=0):
    if elapsed_time > step_time:
        reference = np.array([x,y,yaw])
    else:
        reference = np.array([start_x,start_y,start_yaw])
    return reference

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

            # reference = step_offset(elapsed_time, 20, x=2)

            time_offset = 10
            if elapsed_time < time_offset: # time to stabilize
                reference = np.array([0, 0, 0])
            else:
                reference = figure_eight(3, 2, 100, elapsed_time, time_offset) # eg. figure_eight(2, 1, 45 ...) = 2m width, 1m height, 45s total time

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
            # y_tracker = np.clip(y_tracker, -30, 30) ------ causes maximum distance for reference tracking
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
                    "reference": {"x": float(reference[0]),"y": float(reference[1]), "yaw": math.degrees(float(reference[2]))},
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
        
