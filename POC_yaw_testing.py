import time

import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import syncCrazyflie# type: ignore
import traceback
import math 
import matplotlib.pyplot as plt

from queue import Queue
import threading
from collections import namedtuple
import numpy as np

import pygame

VICON_TRACKER_IP = "192.168.10.1"
OBJECT_NAME = "pipeCrazyflie"

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
    

    roll = 0
    pitch = 0

    current_pos = np.array([0,0,0,0,0,0])
    prev_pos = np.array([0,0,0,0,0,0])

    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()
    
    
    try:
        

        
        offset = get_pos(mytracker.get_position(OBJECT_NAME), current_pos)
        
        left_drone_offset = np.array([0,0,0,0,0,0])
        left_drone_offset = get_pos(mytracker.get_position(LEFT_DRONE_NAME), left_drone_offset)
        right_drone_offset = np.array([0,0,0,0,0,0])
        right_drone_offset = get_pos(mytracker.get_position(RIGHT_DRONE_NAME), right_drone_offset)
        left_drone_pos = get_pos(mytracker.get_position(LEFT_DRONE_NAME), left_drone_offset) - left_drone_offset
        right_drone_pos = get_pos(mytracker.get_position(RIGHT_DRONE_NAME), right_drone_offset) - right_drone_offset
        
        
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
        controlQueues[1].put(Takeoff( height, 3))
        time.sleep(3)
        
        flying = True
        start_time = time.time()
        
        current_pos = get_pos(mytracker.get_position(OBJECT_NAME), current_pos) - offset
        prev_pos = current_pos
        mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
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
            current_pos = get_pos(mytracker.get_position(OBJECT_NAME), current_pos) - offset
            for i in range(len(current_pos)):
                if abs(current_pos[i]) > POSITION_THRESHOLD:
                    current_pos[i] = prev_pos[i]  # Ignore the current velocity component if it exceeds the threshold            
            
            
            current_vel = (np.subtract(current_pos,prev_pos)) / d_time
            current_vel[5] = (current_vel[5] + prev_vel[5])/ 4
            # Apply the filter
            # Apply the filter on a per-component basis
            for i in range(len(current_vel)):
                if abs(current_vel[i]) > VELOCITY_THRESHOLD:
                    current_vel[i] = prev_vel[i]  # Ignore the current velocity component if it exceeds the threshold
            # print("POS:" ,current_pos ,'\n')
            # print("VEL:", current_vel,'\n')
            prev_pos = current_pos
            prev_vel = current_vel
            current_time = time.time()
            elapsed_time = current_time - start_time
            
        
            x = np.array([current_pos[0],current_vel[0], current_pos[1],current_vel[1], current_pos[5],current_vel[5]])
            Cx = np.array([current_pos[0], current_pos[1], current_pos[5]])
            
            # calculate the control output
            
            y_tracker += (Kr @ (reference - Cx))*d_time
            #  Try without reference tracking first!
            u = -Kx @ x #+ y_tracker
            # print("WRENCH CONTROLL:", u)
            yaw = current_pos[5]


            ## split the sum of forces into the the agents compomonents
            HT_thrusts = np.array([u[0], u[1]])
            
            
            #yaw P control (hopefully works well enough)
            left_drone_pos = get_pos(mytracker.get_position(LEFT_DRONE_NAME), left_drone_pos) - left_drone_offset
            right_drone_pos = get_pos(mytracker.get_position(RIGHT_DRONE_NAME), right_drone_pos) - right_drone_offset
            yawrate1 = 1.4 * (x[4] - left_drone_pos[5])
            yawrate2 = 1.4 * (x[4] - right_drone_pos[5])
            yaw_track_history.append([math.degrees(left_drone_pos[5]),math.degrees(right_drone_pos[5]), math.degrees(x[4])])
            yaw_history.append([math.degrees(yawrate1), math.degrees(yawrate2)])
            
            rot_yaw = yaw
            if ((abs(rot_yaw - left_drone_pos[5]) > YAW_THRESHOLD) or (abs(rot_yaw - right_drone_pos[5]) > YAW_THRESHOLD)):
                rot_yaw = (left_drone_pos[5] + left_drone_pos[5]) / 2
                print('YAW DESYNC','\n')
            
            # generate rotation matrix
            rot_matrix = np.array([[np.cos(rot_yaw), np.sin(rot_yaw)],
                [-np.sin(rot_yaw), np.cos(rot_yaw)]])
            # multiply the X Y forces by the rotation matrix
            HT_thrusts = rot_matrix @ HT_thrusts
            bx_thrust = HT_thrusts[0]
            by_thrust = HT_thrusts[1]
            # split the x based on the weighting
            bx_1 = WEIGHTING * bx_thrust
            bx_2 = (1-WEIGHTING) * bx_thrust            
            
            # apply gain to the moment setpoint (1/beamlenght)
            moment_z = u[2]/(2*beam_length)
            # combine the y and yaw
            by_1 = 0.5 * by_thrust
            by_2 = 0.5 * by_thrust
            
            roll_1 = 0.00436332*bx_1
            roll_2 = 0.00436332*bx_2
            
            pitch_1 = -0.00436332*(-by_1 - moment_z) # right one
            pitch_2 = -0.00436332*(-by_2 + moment_z) # left one
            
            
            roll_1 = max(min(roll_1, cap_limit), -cap_limit)
            roll_2 = max(min(roll_2, cap_limit), -cap_limit)

            # Cap the pitch values to ±10 degrees
            pitch_1 = max(min(pitch_1, cap_limit), -cap_limit)
            pitch_2 = max(min(pitch_2, cap_limit), -cap_limit)
        
            # print("Control input1: ", [math.degrees(roll_1), math.degrees(pitch_1), math.degrees(yawrate2), height],'\n')
            # print("Control input1: ", [math.degrees(roll_2), math.degrees(pitch_2), math.degrees(yawrate1), height],'\n')
            
            # send to drones
            controlQueues[0].put(Altitude(math.degrees(roll_1), math.degrees(pitch_1), math.degrees(yawrate2), height)) #ATLAS RIGHT
            controlQueues[1].put(Altitude(math.degrees(roll_2), math.degrees(pitch_2), math.degrees(yawrate1), height)) #PBODY LEFT
            
            x_history.append(x)
            u_history.append(u)
            timeT.append(elapsed_time)
            setpoint_history.append([math.degrees(roll_1), math.degrees(pitch_1),math.degrees(roll_2), math.degrees(pitch_2)])

            # print(d_time)
            time.sleep(0.001) # 100hz
        
        controlQueues[0].put(Land(3))
        controlQueues[1].put(Land(3))
    except Exception as e:
        for ctrl in controlQueues:
            ctrl.put(Quit())
        print(traceback.format_exc())

def update_crazy_controller(scf):
    control = controlQueues[uris.index(scf.cf.link_uri)]
    while True:
        command = control.get()
        if type(command) is Quit:
            scf.cf.high_level_commander.stop()
            return
        elif type(command) is Takeoff:
            scf.cf.high_level_commander.takeoff(command.height, command.time)
        elif type(command) is Land:
            scf.cf.high_level_commander.land(0.0, command.time)
        elif type(command) is Altitude:
            scf.cf.commander.send_zdistance_setpoint(command.roll, command.pitch, command.yawrate, command.altitude)
        else:
            scf.cf.high_level_commander.stop()
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



# EMERGENCY STOP
def stop(scf):
    commander= scf.cf.high_level_commander
    commander.stop()
    print('EMERGENCY STOP OCCURED')
    
def hold_pos(scf):
    commander= scf.cf.high_level_commander
    commander.go_to(0, 0, 0, 0, 1.0, relative=True)
    time.sleep(2)

def get_pos(position, current_pos):
    if len(position[2]) > 0:
        obj_data = position[2][0]
        x, y, z = obj_data[2], obj_data[3], obj_data[4]
        x_rot, y_rot, z_rot  = obj_data[5], obj_data[6], obj_data[7]
        return np.array([x/1000, y/1000, z/1000, x_rot, y_rot, -z_rot])
    else:
        print("LOST SIGHT OF OBJECT")
        return current_pos
                


if __name__ == '__main__':
    controlQueues = [Queue() for _ in range(len(uris))]
        
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    
    with Swarm(uris, factory=factory) as swarm:
        try:    
            swarm.parallel_safe(light_check)
            swarm.reset_estimators()
            
            threading.Thread(target=control_thread).start()

            swarm.parallel_safe(update_crazy_controller)
            
        except Exception as e:
            swarm.parallel_safe(stop)
            print(traceback.format_exc())
        
    pygame.quit()
    # Convert results to numpy arrays for easier handling
    
    x_history = np.array(x_history)
    u_history = np.array(u_history)
    ref_history = np.array(ref_history)
    yaw_track_history = np.array(yaw_track_history)
    setpoint_history = np.array(setpoint_history)
    
    print(len(x_history))
    # Print final state and control input
    # print("Final state:", x)
    # print("Final control input:", u)

    # Plotting the results
    plt.figure(figsize=(12, 6))

    # Plot state response
    plt.subplot(5, 1, 1)
    plt.plot(timeT, x_history)
    plt.title('State Response')
    plt.xlabel('Ti me (s)')
    plt.ylabel('State')
    plt.legend(['x1', 'x_vel', 'y','y_vel','phi','phi_vel'])  # Adjust legend based on the number of states

    # Plot control input
    plt.subplot(5, 1, 2)
    plt.plot(timeT, u_history)
    plt.title('Control Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Input')
    plt.legend(['Fx', 'Fy', 'Mz'])  # Adjust legend based on the number of inputs
    
    plt.subplot(5, 1, 3)
    plt.plot(timeT, setpoint_history)
    plt.title('Control Setpoints')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Setpoints')
    plt.legend(['Right Roll', 'Right Pitch', 'Left Roll', 'Left Pitch'])  # Adjust legend based on the number of inputs

    plt.subplot(5, 1, 4)
    plt.plot(timeT, yaw_track_history)
    plt.title('Yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('measured yaw')
    plt.legend(['left drone', 'right drone', 'payload'])  # Adjust legend based on the number of inputs
    
    plt.subplot(5, 1, 5)
    plt.plot(timeT, yaw_history)
    plt.title('Yaw Control Efforts')
    plt.xlabel('Time (s)')
    plt.ylabel('yawrate')
    plt.legend(['left drone', 'right drone'])  # Adjust legend based on the number of inputs

    plt.tight_layout()
    
    np.savetxt("x_history.csv", x_history, delimiter=",")
    np.savetxt("u_history.csv", u_history, delimiter=",")
    np.savetxt("ref_history.csv", ref_history, delimiter=",")
    np.savetxt("yaw_track_history.csv", yaw_track_history, delimiter=",")
    np.savetxt("setpoint_history.csv", setpoint_history, delimiter=",")
    
    plt.show()  
    

        
