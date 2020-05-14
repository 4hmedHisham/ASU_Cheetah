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

def set_angle(angle_indx,val):
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

    if(angle_indx == 0):
        ab1_pub.publish(val)
    elif(angle_indx == 1):
        bc1_pub.publish(val)
    elif(angle_indx == 2):
        cd1_pub.publish(val)        
    elif(angle_indx == 3):
        ab2_pub.publish(val)
    elif(angle_indx == 4):
        bc2_pub.publish(-val)
    elif(angle_indx == 5):
        cd2_pub.publish(val)
    elif(angle_indx == 6):
        ab3_pub.publish(val)
    elif(angle_indx == 7):
        bc3_pub.publish(-val)
    elif(angle_indx == 8):
        cd3_pub.publish(val)        
    elif(angle_indx == 9):
        ab4_pub.publish(val)
    elif(angle_indx == 10):
        bc4_pub.publish(val)
    elif(angle_indx == 11):
        cd4_pub.publish(val)


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


# Main
if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('setter', String, queue_size=10)
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
    rate = rospy.Rate(100)  # 10hz
    time.sleep(0.5)
    init_angles()  
    while not rospy.is_shutdown():

        rate.sleep()



