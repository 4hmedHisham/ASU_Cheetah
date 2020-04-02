#!/usr/bin/env python

######imports#############

import rospy
import numpy
import torch
import math
from std_msgs.msg import Float32MultiArray, Float32
from timeit import default_timer as timer

######Global Variables####

Cutoff = 1

IP = numpy.zeros(2)
GM = numpy.zeros(2)

Integral = numpy.zeros(2)
Resid = numpy.zeros(2)

Integral.astype(float)
Resid.astype(float)
IP.astype(float)
GM.astype(float)


time = float(0)
time_prev = float(0)

######Topics ISR##########

def GeneralizedMomentum(data):
    global GM
    GM = numpy.array(data.data)

def IntegralPart(data):
    global IP
    IP = numpy.array(data.data)  

######Main################
if __name__ == '__main__':
    rospy.init_node('ContactDetector', anonymous=False)
    rospy.Subscriber('GM', Float32MultiArray , GeneralizedMomentum)
    rospy.Subscriber('IntegralPart', Float32MultiArray , IntegralPart)
    pub = rospy.Publisher('SwingStance', Float32, queue_size = 2)
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        
        start = timer()
      
        time_prev = time
        time = timer()  

        Integral = Integral + (IP + Resid) * (time - time_prev)    
        Resid = Cutoff * (GM - Integral)

        pblshd = Float32(round(Resid[0]/(1000*1000*1000),2))#,round(Resid[1]/(1000*1000*1000),2)]
        pub.publish(pblshd)
        
        end = timer()
        #print(end-start)
        rate.sleep()
