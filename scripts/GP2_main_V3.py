#!/usr/bin/env python
# Description : Creeping and trotting motion  2d inv kinematics only
#changed client id to zero review this
#import matplotlib.pyplot as plt
import sys
import math
import numpy as np
import time
import vrep
#import GP2_Function_V3 as  
#changed V3 with omar updated V7
from Main_Functions import GP2_Function_V7 as gait
import GP2_Vrep_V3 as v


# Naming Conevention For Legs
# TopLeft 1   Top Right 2 Rear Right 3 RearLeft 4
gait.ros.ros_init()
time.sleep(3)
print("start")

################### MAIN #####################

## el 2 lines dol 3alashan el sudden motion ale by3melha fel awel

gait.Cheetah_Wakeup()
gait.Cheetah_Sleep()

gait.onestepcreeping('f',120)
time.sleep(0.5)
gait.Body_mover_To_point(0,gait.a,-gait.initalheight,0.01) # el line dah byraga3 el joints lel initial position
time.sleep(0.5)





time.sleep(0.5)

for i in range(20):
    gait.onestepcreeping('f',180)
    time.sleep(0.5)




# for i in range(5):
#     gait.Move_side("l")
#     time.sleep(0.5)
# transverses , hips, knees = gait.getjointanglesfromvrep()
# legspos2cg,legspos2joint=gait.GetEndEffectorPos(transverses,hips,knees)#
# print(legspos2cg)
# print(legspos2joint)











