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
time.sleep(2)

#a,b,c = gait.inverse_kinematics_3d_v6(0,gait.a , -390 , 0,-2 ,1)


gait.Dancing()

for i in range(20):
    gait.onestepcreeping('f',180)
    time.sleep(0.5)







