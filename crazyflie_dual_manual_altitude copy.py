import time

import cflib.crtp# type: ignore
from cflib.crazyflie.swarm import CachedCfFactory# type: ignore
from cflib.crazyflie.swarm import Swarm# type: ignore
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import syncCrazyflie# type: ignore
import traceback
import pyvicon_datastream as pv
from pyvicon_datastream import tools
import math 
import matplotlib.pyplot as plt


VICON_TRACKER_IP = "192.168.10.1"
OBJECT_NAME = "pipeCrazyflie"

LEFT_DRONE_NAME = "PbodyCrazyFlie"
RIGHT_DRONE_NAME = "AtlasCrazyFlie"
YAW_THRESHOLD = 0.5
VELOCITY_THRESHOLD = 10.0 
WEIGHTING = 0.5
beam_length = 1
import numpy as np

import pygame

uris = [
    'radio://0/80/2M/E7E7E7E7E7', #Atlas ON THE RIGHT
    'radio://0/81/2M/E6E7E6E7E6', #P-Body ON THE LEFT
]

reference = np.array([0, 0, 0])  # reference state
height = 0.6


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
        return np.array([x/1000, y/1000, z/1000, x_rot, y_rot, z_rot])
    else:
        print("LOST SIGHT OF OBJECT")
        return current_pos
                


if __name__ == '__main__':

    vicon_client = pv.PyViconDatastream()
    ret = vicon_client.connect(VICON_TRACKER_IP)
    mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
    
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    pygame.init()
    pygame.joystick.init()
    controller = pygame.joystick.Joystick(0)
    controller.init()
    
    roll = 0
    pitch = 0
    yawrate = 0

    current_pos = np.array([0,0,0,0,0,0])
    prev_pos = np.array([0,0,0,0,0,0])
    
    
    x_history = []
    u_history = []  
    ref_history = []
    full_history = []
    yaw_history = []
    yaw_track_history = []
    timeT = []

    Kr = matrix = np.array([
        [3.46410161513777, -1.77635683940025e-15 , 9.69580370540695e-16],
        [-1.33226762955019e-15, 3.46410161513776, 1.87943008165182e-15],
        [-1.26082045570467e-13 ,1.83959217624210e-14 ,0.999999999999995]
    ])

    Kx = np.array([
        [2.20073124711112  ,  0.699032773431331   , -1.44684744558436e-15  ,  2.22044604925031e-16   , 1.44487317823154e-15 ,   1.88088314185855e-16],
        [-1.36236425151406e-15   , 7.19098461534927e-16  ,  2.20073124711111  ,  0.699032773431330  ,  1.87346911436869e-15  ,  6.17180657674190e-16],
        [-5.14970241659517e-14  ,  -2.53198500656967e-14  ,  6.37121548776897e-15  ,  1.03094031648370e-15   , 0.462530140083379  ,  0.106465947992776]
    ])

    with Swarm(uris, factory=factory) as swarm:
        try:
            if ret != pv.Result.Success:
                raise Exception('Connection to {VICON_TRACKER_IP} failed')
            else:
                print(f"Connection to {VICON_TRACKER_IP} successful")
            
            
            
            swarm.parallel_safe(light_check)
            swarm.reset_estimators()
            
            
            offset = get_pos(mytracker.get_position(OBJECT_NAME), current_pos)
            
            left_drone_offset = np.array([0,0,0,0,0,0])
            left_drone_offset = get_pos(mytracker.get_position(LEFT_DRONE_NAME), left_drone_offset)
            right_drone_offset = np.array([0,0,0,0,0,0])
            right_drone_offset = get_pos(mytracker.get_position(RIGHT_DRONE_NAME), right_drone_offset)
            
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
            
            swarm.parallel_safe(take_off)
            
            flying = True
            start_time = time.time()
            
            current_pos = get_pos(mytracker.get_position(OBJECT_NAME), current_pos) - offset
            prev_pos = current_pos
            mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
            current_time = time.time()
            prev_time = current_time
            left_drone_pos = get_pos(mytracker.get_position(LEFT_DRONE_NAME), left_drone_offset) - left_drone_offset
            right_drone_pos = get_pos(mytracker.get_position(RIGHT_DRONE_NAME), right_drone_offset) - right_drone_offset
            
            
            y_tracker = 0
  
            prev_vel = np.array([0, 0, 0,0,0,0])
            print('starting altitude flight')
            while flying:
            
                # safety timeout
                elapsed_time = time.time() - start_time

                if elapsed_time > 5:
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
                current_vel = (np.subtract(current_pos,prev_pos)) / d_time
                # Apply the filter
                # Apply the filter on a per-component basis
                for i in range(len(current_vel)):
                    if abs(current_vel[i]) > VELOCITY_THRESHOLD:
                        current_vel[i] = prev_vel[i]  # Ignore the current velocity component if it exceeds the threshold
                print("POS:" ,current_pos ,'\n')
                print("VEL:", current_vel,'\n')
                prev_pos = current_pos
                prev_vel = current_vel
                current_time = time.time()
                elapsed_time = current_time - start_time
                
            
                x = np.array([current_pos[0],current_vel[0], current_pos[1],current_vel[1], current_pos[5],current_vel[5]])
                Cx = np.array([current_pos[0], current_pos[1], current_pos[5]])
                
                # calculate the control output
                
                y_tracker += Kr @ (reference - Cx)
                #  Try without reference tracking first!
                u = -Kx @ x #+ y_tracker
                print("WRENCH CONTROLL:", u)
                yaw = current_pos[5]


                ## split the sum of forces into the the agents compomonents
                HT_thrusts = np.array([u[0], u[1]])
                
                
                #yaw P control (hopefully works well enough)
                left_drone_pos = get_pos(mytracker.get_position(LEFT_DRONE_NAME), left_drone_pos) - left_drone_offset
                right_drone_pos = get_pos(mytracker.get_position(RIGHT_DRONE_NAME), right_drone_pos) - right_drone_offset
                yawrate1 = 0.1 * (x[4] - left_drone_pos[5])
                yawrate2 = 0.1 * (x[4] - right_drone_pos[5])
                yaw_track_history.append([math.degrees(left_drone_pos[5]),math.degrees(right_drone_pos[5]), math.degrees(x[4])])
                yaw_history.append([math.degrees(yawrate1), math.degrees(yawrate2)])
                
                
                
                rot_yaw = yaw
                if ((abs(rot_yaw - left_drone_pos[5]) > YAW_THRESHOLD) or (abs(rot_yaw - right_drone_pos[5]) > YAW_THRESHOLD)):
                    rot_yaw = left_drone_pos[5] + left_drone_pos[5] / 2
                
                # generate rotation matrix
                rot_matrix = np.array([[np.cos(rot_yaw), -np.sin(rot_yaw)],
                    [np.sin(rot_yaw), np.cos(rot_yaw)]])
                # multiply the X Y forces by the rotation matrix
                HT_thrusts = HT_thrusts @  rot_matrix
                bx_thrust = HT_thrusts[0]
                by_thrust = HT_thrusts[1]
                # split the x based on the weighting
                bx_1 = WEIGHTING * bx_thrust
                bx_2 = (1-WEIGHTING) * bx_thrust            
                
                # apply gain to the moment setpoint (1/beamlenght)
                moment_z = u[2]/(50*beam_length)
                # combine the y and yaw
                by_1 = WEIGHTING * by_thrust 
                by_2 = (1-WEIGHTING) * by_thrust
                
                roll_1 = -0.25*bx_1
                roll_2 = -0.25*bx_2
                
                pitch_1 = -0.025*by_1 - moment_z
                pitch_2 = -0.025*by_2 + moment_z
                

                
                
                print("Control input1: ", [math.degrees(roll_1), math.degrees(pitch_1), math.degrees(yawrate2), height],'\n')
                print("Control input1: ", [math.degrees(roll_2), math.degrees(pitch_2), math.degrees(yawrate1), height],'\n')
                
                # send to drones
                args_dict = {
                    uris[0]: [math.degrees(roll_1), math.degrees(pitch_1), math.degrees(yawrate2), height], # RIGHT DRONE (ATLAS)
                    uris[1]: [math.degrees(roll_2), math.degrees(pitch_2), math.degrees(yawrate1), height], # LEFT DRONE (PBODY)
                }
                swarm.parallel_safe(update_controller, args_dict = args_dict)

                x_history.append(x)
                u_history.append(u)
                timeT.append(elapsed_time)

                
                time.sleep(0.01) # 100hz

            swarm.parallel_safe(land)
        except Exception as e:
            swarm.parallel(stop)
            print(traceback.format_exc())

    pygame.quit()
    # Convert results to numpy arrays for easier handling
    x_history = np.array(x_history)
    u_history = np.array(u_history)
    ref_history = np.array(ref_history)
    yaw_track_history = np.array(yaw_track_history)
    # Print final state and control input
    # print("Final state:", x)
    # print("Final control input:", u)

    # Plotting the results
    plt.figure(figsize=(12, 6))

    # Plot state response
    plt.subplot(4, 1, 1)
    plt.plot(timeT, x_history)
    plt.title('State Response')
    plt.xlabel('Time (s)')
    plt.ylabel('State')
    plt.legend(['x1', 'x_vel', 'y','y_vel','phi','phi_vel'])  # Adjust legend based on the number of states

    # Plot control input
    plt.subplot(4, 1, 2)
    plt.plot(timeT, u_history)
    plt.title('Control Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Input')
    plt.legend(['Fx', 'Fy', 'Mz'])  # Adjust legend based on the number of inputs

    plt.subplot(4, 1, 3)
    plt.plot(timeT, yaw_track_history)
    plt.title('Yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('measured yaw')
    plt.legend(['left drone', 'right drone', 'payload'])  # Adjust legend based on the number of inputs
    
    plt.subplot(4, 1, 4)
    plt.plot(timeT, yaw_history)
    plt.title('Yaw Control Efforts')
    plt.xlabel('Time (s)')
    plt.ylabel('yawrate')
    plt.legend(['left drone', 'right drone'])  # Adjust legend based on the number of inputs

    plt.tight_layout()
    plt.show()  
        
