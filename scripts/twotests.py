# -*- coding: utf-8 -*-
#!/usr/bin/env python
"""
Created on Sun Feb  9 13:02:41 2020

@author: ShadowZone
"""
import sim
import numpy as np
import platform
import time
print(platform.python_version())
clientID=0
def get_angles_firsttime():
    for i in range(12):
        print('handler is')
        print(int(angles_handler[i]))
        trash,ang = sim.simxGetJointPosition(clientID, int(angles_handler[i]), sim.simx_opmode_streaming)
        res=sim.simx_return_novalue_flag
        while res!=sim.simx_return_ok:
            res,ang1=sim.simxGetJointPosition(clientID,int(angles_handler[i]),sim.simx_opmode_buffer)
        print("Angle x IS")
        print(ang1)
def get_torques_firsttime():
    for i in range(12):
        print('handler is')
        print(int(angles_handler[i]))
        trash,torque = sim.simxJointGetForce(clientID, int(angles_handler[i]), sim.simx_opmode_streaming)
        res=sim.simx_return_novalue_flag
        while res!=sim.simx_return_ok:
            res,torque1=sim.simxGetJointForce(clientID,int(angles_handler[i]),sim.simx_opmode_buffer)
        print("TORQUE x IS")
        print(torque1)


def get_torque(joint):
    angles = []
    error = []
    if joint == 'ab3' or joint == 0:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[0]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'bc3' or joint == 1:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[1]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'cd3' or joint == 2:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[2]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'ab4' or joint == 3:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[3]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'bc4' or joint == 4:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[4]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'cd4' or joint == 5:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[5]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'ab1' or joint == 6:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[6]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'bc1' or joint == 7:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[7]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'cd1' or joint == 8:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[8]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'ab2' or joint == 9:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[9]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'bc2' or joint == 10:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[10]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif joint == 'cd2' or joint == 11:
        trash, torque = sim.simxGetJointForce(clientID, int(angles_handler[11]), sim.simx_opmode_buffer)
        # angels.append(ang)
    return torque



def get_angles(getangles):
    angle = getangles
    angles = []
    error = []
    if angle == 'ab3' or angle == 0:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[0]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'bc3' or angle == 1:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[1]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'cd3' or angle == 2:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[2]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'ab4' or angle == 3:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[3]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'bc4' or angle == 4:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[4]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'cd4' or angle == 5:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[5]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'ab1' or angle == 6:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[6]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'bc1' or angle == 7:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[7]), sim.simx_opmode_buffer)#from simx_opmode_buffer

        # angels.append(ang)
    elif angle == 'cd1' or angle == 8:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[8]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'ab2' or angle == 9:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[9]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'bc2' or angle == 10:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[10]), sim.simx_opmode_buffer)
        # angels.append(ang)
    elif angle == 'cd2' or angle == 11:
        trash, ang = sim.simxGetJointPosition(clientID, int(angles_handler[11]), sim.simx_opmode_buffer)
        # angels.append(ang)
    
    return ang


def set_angle(setangle, angle):
    if setangle == 'ab3' or setangle == 0:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[0]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'bc3' or setangle == 1:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[1]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'cd3' or setangle == 2:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[2]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'ab4' or setangle == 3:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[3]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'bc4' or setangle == 4:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[4]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'cd4' or setangle == 5:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[5]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'ab1' or setangle == 6:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[6]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'bc1' or setangle == 7:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[7]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'cd1' or setangle == 8:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[8]), angle, sim.simx_opmode_oneshot)
        print('went through here with an error ')
    elif setangle == 'ab2' or setangle == 9:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[9]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'bc2' or setangle == 10:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[10]), angle, sim.simx_opmode_oneshot)
    elif setangle == 'cd2' or setangle == 11:
        sim.simxSetJointTargetPosition(clientID, int(angles_handler[11]), angle, sim.simx_opmode_oneshot)


def vrepInterface(port):
    global angles_handler
    angles_handler = np.zeros(12)
    global angles_error
    angles_error = np.zeros(12)
    #global clientID
    #clientID = 0
    global inital_name
    print('Program started')
    sim.simxFinish(-1)  # just in case, close all opened connections
    ID = sim.simxStart('127.0.0.1', port, True, True, 5000, 5)  # Connect to V-REP
    print('client id is ' +str(ID))
    if ID != -1:
        print('Connected to remote API server')

        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res, objs = sim.simxGetObjects(ID, sim.sim_handle_all, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print('Number of objects in the scene: ', len(objs))
        else:
            print('Remote API function call returned with error code: ', res)
    else:
        print('DIDNOT CONNECT!!!')

    intial_name = ['ab3', 'bc3', 'cd3', 'ab4', 'bc4', 'cd4', 'ab1', 'bc1', 'cd1', 'ab2', 'bc2', 'cd2']
    for i in range(angles_handler.shape[0]):
        angles_error[i], angles_handler[i] = sim.simxGetObjectHandle(ID, intial_name[i],
                                                                      sim.simx_opmode_blocking)
    return ID

vrepInterface(19960)
ang=0
print('ang is '+str(ang))
set_angle('cd1',ang)



