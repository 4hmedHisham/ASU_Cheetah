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







#gait.Body_mover_To_point(40,gait.a,-200,0.1 )
for i in range(12):
    gait.onestepcreeping('b',120)
    time.sleep(0.5)

# gait.Body_mover('f',0.01,90)
# time.sleep(0.5)
# gait.Body_mover('b',0.01,90)
# time.sleep(0.5)
# gait.Body_mover('l',0.005,90)
# time.sleep(0.5)
# gait.Body_mover('r',0.005,180)
# time.sleep(0.5)
# gait.Body_mover('l',0.005,90)
# time.sleep(0.5)
# gait.Body_mover('d',0.02,200)
# time.sleep(0.5)
# gait.Body_mover('u',0.02,200)
# time.sleep(0.5)


time.sleep(1)
gait.Body_mover('d',0.02,200)
time.sleep(2)
gait.Body_mover('u',0.02,200)

for i in range(10):
    gait.Move_side('l')
    time.sleep(0.3)
gait.move_2_legs(0,3,'l',90)





