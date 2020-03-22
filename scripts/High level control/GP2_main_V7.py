# Description : First trial ene asha8al el transverse
#import matplotlib.pyplot as plt
import sys
import math
import numpy as np
import time
import vrep
import GP2_Function_V7 as gait
import GP2_Vrep_V7 as v

# Parameters

clientID = 0
# Naming Conevention For Legs
# TopLeft 1   Top Right 2 Rear Right 3 RearLeft 4

# Starting Vrep Interface
v.vrepInterface()
v.get_angles_firsttime()
#v.Get_Torques_Firsttime()

# Geting the joint handlers
angles_handler = np.zeros(12)
angles_error = np.zeros(12)
intial_name = ['ab3', 'bc3', 'cd3', 'ab4', 'bc4', 'cd4', 'ab1', 'bc1', 'cd1', 'ab2', 'bc2', 'cd2']
torquehip = np.zeros((4, 100))
torqueknee = np.zeros((4, 100))
iteration = np.zeros(100)
for i in range(angles_handler.shape[0]):
    angles_error[i], angles_handler[i] = vrep.simxGetObjectHandle(gait.clientID, intial_name[i],
                                                                  vrep.simx_opmode_blocking)




for i in range(10):
    gait.Move_side('l')
    time.sleep(0.3)
gait.move_2_legs(0,3,'l',90)

for i in range(25):
    gait.onestepcreeping('f',120)

# for i in range(10):
#     gait.One_Trot('f',120)






#
# for i in range(5):
#     gait.Move_side('r')
#     time.sleep(0.3)



print("finished")

