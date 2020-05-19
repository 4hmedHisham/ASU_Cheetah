#!/usr/bin/env python
import rospy
import numpy as np
import sympy as sp
import math
from numpy import sin , cos
import scipy.linalg
from sympy.solvers import solve
from sympy import Symbol, Eq
from timeit import default_timer as time
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from Main_Functions import GP2_Function_V7 as gait
import matplotlib.pyplot as plt

l1 =245
l2 =208.4
initial_leg_height = 390 # from ground to joint   320
stride = 150
ct=1.2
h = 100     # maximum height of trajectory
cycle_time =ct  # total time for each cycle
steps = 20 # number of steps 'even'
initial_distance = 0 # along x
stride_f = 150
h_f = 100
cycle_time_f = ct
initial_distance_f = None
sample_time_f = None
pub = 0
indx_fix = 0

flag_start =0
x_current = 0 
y_current = 0
x_current_f =0
t_left = 0
current = 0
last_fix = 0
sample_time = np.float(cycle_time) / steps # sample time
sample_time_f =np.float(cycle_time_f) / steps   # sample time
stride= 300
stride_f= 300

# xnew_multi = np.zeros([steps, 1], dtype=float)
# ynew_multi = np.zeros([steps, 1], dtype=float)
# for j in range(4):
#     t = 0
#     i = 0
#     xnew = np.zeros([steps, 1], dtype=float)

#     for t in np.arange(0, cycle_time_f, sample_time_f):
#         xnew[i] = (stride_f * ((t / cycle_time_f) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time_f)))) - (stride_f / 2) + stride_f / 2) #+ initial_distance_f
#         i = i + 1

#     i = 0
#     ynew = np.zeros([steps, 1], dtype=float)

#     # First part of Ynew in piecewise
#     for t in np.arange(0, cycle_time_f / 2, sample_time_f):
#         ynew[i] = (-(h_f / (2 * np.pi)) * np.sin(((4 * np.pi) / cycle_time_f) * t) + ((2 * h_f * t) / cycle_time_f) - (h_f / 2)) + (h_f / 2) #- initial_leg_height_f
#         i = i + 1

#     n = (cycle_time_f / 2)
#     for t in np.arange(n, cycle_time_f, sample_time_f):
#         ynew[i] = (-(h_f / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time_f) * t)) - ((2 * h_f * t) / cycle_time_f) + ((3 * h_f) / 2)) + (h_f / 2) #- initial_leg_height_f
#         i = i + 1
    
#     xnew_multi = np.append(xnew_multi, xnew , axis=1)
#     ynew_multi = np.append(ynew_multi, ynew , axis=1)
# xnew_multi=np.transpose(xnew_multi)
# ynew_multi=np.transpose(ynew_multi)
# #print(xnew_multi)
# l=len(xnew_multi[0])
# for j in range(l):
#     x=xnew_multi[j]
#     y=ynew_multi[j]
#     plt.plot(x,y)
#     plt.show()

result2=gait.forward_kinematics_V3(0,-2.099,1.108)
result0=gait.forward_kinematics_V3(0,-2.058,1.071)
result1=gait.forward_kinematics_V3(0,-2.037,1.140)
print("---------------------")