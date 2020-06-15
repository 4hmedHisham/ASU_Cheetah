#!/usr/bin/env python

import rospy
import numpy as np
import sympy as sp
import math
from numpy import sin , cos
# import torch
import scipy.linalg
from sympy.solvers import solve
from sympy import Symbol, Eq

#from sympy.solvers.solveset import linsolve
from timeit import default_timer as time
import time
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState






hipinit=2.0589882059651674   #   was HIP           -2.27999663185
kneeinit=1.0711307090938094  #KNEE WAS       1.57918160492
ab1_pub = 0
bc1_pub = 0
cd1_pub = 0
ab2_pub = 0
bc2_pub = 0
cd2_pub = 0
ab3_pub = 0
bc3_pub = 0
cd3_pub = 0
ab4_pub = 0
bc4_pub = 0
cd4_pub = 0
msg =[]

def angles(data):
    global msg
    msg = data.position
    #print(msg[0])


def set_angle(angle,val):
    global ab1_pub
    global bc1_pub
    global cd1_pub
    global ab2_pub
    global bc2_pub
    global cd2_pub
    global ab3_pub
    global bc3_pub
    global cd3_pub
    global ab4_pub
    global bc4_pub
    global cd4_pub

    if angle == 'ab3' or angle == 0:
        ab3_pub.publish(val)
    elif angle == 'bc3' or angle == 1:
        bc3_pub.publish(-val)
    elif angle == 'cd3' or angle == 2:
        cd3_pub.publish(val)
    elif angle == 'ab4' or angle == 3:
        ab4_pub.publish(val)
    elif angle == 'bc4' or angle == 4:
        bc4_pub.publish(val)
    elif angle == 'cd4' or angle == 5:
        cd4_pub.publish(val)
    elif angle == 'ab1' or angle == 6:
        ab1_pub.publish(val)
    elif angle == 'bc1' or angle == 7:
        bc1_pub.publish(val)
    elif angle == 'cd1' or angle == 8:
        cd1_pub.publish(val)
    elif angle == 'ab2' or angle == 9:
        ab2_pub.publish(val)
    elif angle == 'bc2' or angle == 10:
        bc2_pub.publish(-val)
    elif angle == 'cd2' or angle == 11:
        cd2_pub.publish(val)


def get_angles(angle):

    global msg    
    if angle == 'ab3' or angle == 0:
        ang = msg[2]
    elif angle == 'bc3' or angle == 1:
        ang = msg[6]
    elif angle == 'cd3' or angle == 2:
        ang = msg[10]
    elif angle == 'ab4' or angle == 3:
        ang = msg[3]
    elif angle == 'bc4' or angle == 4:
        ang = msg[7]
    elif angle == 'cd4' or angle == 5:
        ang = msg[11]
    elif angle == 'ab1' or angle == 6:
        ang = msg[0]
    elif angle == 'bc1' or angle == 7:
        ang = msg[4]
    elif angle == 'cd1' or angle == 8:
        ang = msg[8]
    elif angle == 'ab2' or angle == 9:
        ang = msg[1]
    elif angle == 'bc2' or angle == 10:
        ang = msg[5]
    elif angle == 'cd2' or angle == 11:
        ang = msg[9]  
    return ang


def init_angles():
        set_angle(0,0)
        time.sleep(0.2)
        set_angle(1,hipinit)
        time.sleep(0.2)       
        set_angle(2,kneeinit)
        time.sleep(0.2)
        set_angle(3,0)
        time.sleep(0.2)
        set_angle(4,hipinit)
        time.sleep(0.2) 
        set_angle(5,kneeinit)
        time.sleep(0.2)
        set_angle(6,0)
        time.sleep(0.2)        
        set_angle(7,hipinit)
        time.sleep(0.2)
        set_angle(8,kneeinit)
        time.sleep(0.2)
        set_angle(9,0)
        time.sleep(0.2)
        set_angle(10,hipinit)
        time.sleep(0.2)
        set_angle(11,kneeinit)
        time.sleep(0.2)
        print("Angles initialized")




def ros_init_gaz():
    #rospy.init_node('talker', anonymous=True)
    global ab1_pub
    global bc1_pub
    global cd1_pub
    global ab2_pub
    global bc2_pub
    global cd2_pub
    global ab3_pub
    global bc3_pub
    global cd3_pub
    global ab4_pub
    global bc4_pub
    global cd4_pub
    ab1_pub = rospy.Publisher('/SimplifiedAssembly/ab1_position_controller/command', Float64, queue_size=1)
    bc1_pub = rospy.Publisher('/SimplifiedAssembly/bc1_position_controller/command', Float64, queue_size=1)
    cd1_pub = rospy.Publisher('/SimplifiedAssembly/cd1_position_controller/command', Float64, queue_size=1)
    ab2_pub = rospy.Publisher('/SimplifiedAssembly/ab2_position_controller/command', Float64, queue_size=1)
    bc2_pub = rospy.Publisher('/SimplifiedAssembly/bc2_position_controller/command', Float64, queue_size=1)
    cd2_pub = rospy.Publisher('/SimplifiedAssembly/cd2_position_controller/command', Float64, queue_size=1)
    ab3_pub = rospy.Publisher('/SimplifiedAssembly/ab3_position_controller/command', Float64, queue_size=1)
    bc3_pub = rospy.Publisher('/SimplifiedAssembly/bc3_position_controller/command', Float64, queue_size=1)
    cd3_pub = rospy.Publisher('/SimplifiedAssembly/cd3_position_controller/command', Float64, queue_size=1)
    ab4_pub = rospy.Publisher('/SimplifiedAssembly/ab4_position_controller/command', Float64, queue_size=1)
    bc4_pub = rospy.Publisher('/SimplifiedAssembly/bc4_position_controller/command', Float64, queue_size=1)
    cd4_pub = rospy.Publisher('/SimplifiedAssembly/cd4_position_controller/command', Float64, queue_size=1)
    rospy.Subscriber('/SimplifiedAssembly/joint_states', JointState , angles)
    # time.sleep(0.3)
    # init_angles()