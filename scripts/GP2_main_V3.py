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
for i in range(10):
    gait.Move_side('l')
    time.sleep(0.3)
gait.move_2_legs(0,3,'l',90)

for i in range(25):
    gait.onestepcreeping('f',120)





