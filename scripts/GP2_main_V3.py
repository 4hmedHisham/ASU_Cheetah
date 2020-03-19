# Description : Creeping and trotting motion  2d inv kinematics only
#changed client id to zero review this
import matplotlib.pyplot as plt
import sys
import math
import numpy as np
import time
import vrep
import GP2_Function_V3 as gait
import GP2_Vrep_V3 as v

# Parameters
l1 = 244.59
l2 = 208.4
clientID = 0
# Naming Conevention For Legs
# TopLeft 1   Top Right 2 Rear Right 3 RearLeft 4

# Starting Vrep Interface
v.vrepInterface(19998)
v.get_angles_firsttime()

# Geting the joint handlers
angles_handler = np.zeros(12)
angles_error = np.zeros(12)
intial_name = ['ab3', 'bc3', 'cd3', 'ab4', 'bc4', 'cd4', 'ab1', 'bc1', 'cd1', 'ab2', 'bc2', 'cd2']
torquehip = np.zeros((4, 100))
torqueknee = np.zeros((4, 100))
iteration = np.zeros(100)
for i in range(angles_handler.shape[0]):
    angles_error[i], angles_handler[i] = vrep.simxGetObjectHandle(0, intial_name[i],
                                                                  vrep.simx_opmode_blocking)#changed client id to zero review this
time.sleep(2)
for i in range(10):
    time.sleep(0.5)
    gait.trot2(1)
    time.sleep(0.5)
    gait.trot2(2)
    time.sleep(0.5)





