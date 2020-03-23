
import sim
import numpy as np
import time

def vrepInterface(port):
    global angles_handler
    angles_handler = np.zeros(12)
    global angles_error
    angles_error = np.zeros(12)
    global clientID
    clientID = 0
    global inital_name
    print('Program started')
    sim.simxFinish(-1)  # just in case, close all opened connections
    ID = sim.simxStart('127.0.0.1', port, True, True, 5000, 5)  # Connect to V-REP
    print(ID)
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
        angles_error[i], angles_handler[i] = sim.simxGetObjectHandle(clientID, intial_name[i],
                                                                      sim.simx_opmode_blocking)
    return ID

def imu_read_firsttime():
    error,x=sim.simxGetFloatSignal(0,'accelerometerX',sim.simx_opmode_buffer)
    while(error!=sim.simx_return_ok):
        error,x=sim.simxGetFloatSignal(0,'accelerometerX',sim.simx_opmode_streaming)
    error2,y=sim.simxGetFloatSignal(0,'accelerometerY',sim.simx_opmode_buffer)
    while(error2!=sim.simx_return_ok):
        error2,y=sim.simxGetFloatSignal(0,'accelerometerY',sim.simx_opmode_streaming)
    error3,z=sim.simxGetFloatSignal(0,'accelerometerZ',sim.simx_opmode_buffer)
    while(error3!=sim.simx_return_ok):
        error3,z=sim.simxGetFloatSignal(0,'accelerometerZ',sim.simx_opmode_streaming)

def imu_read():
    error=1
    error2=1
    error3=1
    while(error!=0):
        error,x=sim.simxGetFloatSignal(0,'accelerometerX',sim.simx_opmode_streaming)
    while(error2!=0):
        error2,y=sim.simxGetFloatSignal(0,'accelerometerY',sim.simx_opmode_streaming)
    while(error3!=0):
        error3,z=sim.simxGetFloatSignal(0,'accelerometerZ',sim.simx_opmode_streaming)
    return x,y,z
def gyro_read_firsttime():
    error,x=sim.simxGetFloatSignal(0,'gyroX',sim.simx_opmode_buffer)
    while(error!=sim.simx_return_ok):
        error,x=sim.simxGetFloatSignal(0,'gyroX',sim.simx_opmode_streaming)
    print("LINEAR X IS ")
    print(x)
    error2,y=sim.simxGetFloatSignal(0,'gyroY',sim.simx_opmode_buffer)
    while(error2!=sim.simx_return_ok):
        error2,y=sim.simxGetFloatSignal(0,'gyroY',sim.simx_opmode_streaming)
    print("LINEAR Y IS ")
    print(y)
    error3,z=sim.simxGetFloatSignal(0,'gyroZ',sim.simx_opmode_buffer)
    while(error3!=sim.simx_return_ok):
        error3,z=sim.simxGetFloatSignal(0,'gyroZ',sim.simx_opmode_streaming)
    print("LINEAR Z IS ")
    print(z)

def gyro_read():
    error=1
    error2=1
    error3=1
    while(error!=0):
        error,x=sim.simxGetFloatSignal(0,'gyroX',sim.simx_opmode_streaming)
    print("LINEAR Y IS ")
    print(y)
    while(error2!=0):
        error2,y=sim.simxGetFloatSignal(0,'gyroY',sim.simx_opmode_streaming)
    print("LINEAR Y IS ")
    print(y)
    while(error3!=0):
        error3,z=sim.simxGetFloatSignal(0,'gyroZ',sim.simx_opmode_streaming)
    print("LINEAR Y IS ")
    print(y)
    return x,y,z
vrepInterface(19999)
time.sleep(3)
gyro_read_firsttime()
for i in range(10):
    print("GYRO READ IS ")
    print(gyro_read())
    time.sleep(2)
    