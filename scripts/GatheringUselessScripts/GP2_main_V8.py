# Description : First trial ene asha8al el transverse
#!/usr/bin/env python
#changed client id to zero review this
#import matplotlib.pyplot as plt
import sys
import math
import numpy as np
import time
import vrep
import GP2_Function_V8 as gait
import GP2_Vrep_V8 as v

# Naming Conevention For Legs
# TopLeft 1   Top Right 2 Rear Right 3 RearLeft 4


gait.ros.ros_init()

for i in range(4):
    gait.Move_side('l')

for i in range(25):
    gait.onestepcreeping('f',160)
    time.sleep(0.3)


gait.move_2_legs(0,3,'l',90)


# for i in range(10):
#     gait.One_Trot('f',120)






#
# for i in range(5):
#     gait.Move_side('r')
#     time.sleep(0.3)



print("finished")

