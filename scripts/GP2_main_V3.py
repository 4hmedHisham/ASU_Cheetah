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


# #a,b,c = gait.inverse_kinematics_3d_v6(0,gait.a , -390 , 0,-2 ,1)

# gait.onestepcreeping('f',120)
# time.sleep(1)
# gait.Dancing()

# for i in range(20):
#     gait.onestepcreeping('f',120)
#     time.sleep(0.5)
# transverses , hips, knees = gait.getjointanglesfromvrep()
# legspos2cg,legspos2joint=gait.GetEndEffectorPos(transverses,hips,knees)#
# print(legspos2cg)
# print(legspos2joint)











