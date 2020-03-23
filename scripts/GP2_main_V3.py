#!/usr/bin/env python
# Description : Creeping and trotting motion  2d inv kinematics only
#changed client id to zero review this
#import matplotlib.pyplot as plt
import sys
import math
import numpy as np
import time
import vrep
import GP2_Function_V3 as gait
import GP2_Vrep_V3 as v


# Naming Conevention For Legs
# TopLeft 1   Top Right 2 Rear Right 3 RearLeft 4
gait.ros.ros_init()
time.sleep(2)
for i in range(10):
    time.sleep(0.5)
    gait.trot2(1)
    time.sleep(0.5)
    gait.trot2(2)
    time.sleep(0.5)





