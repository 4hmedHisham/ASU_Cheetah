#!/usr/bin/env python

######imports#############

import rospy
import numpy
import torch
import math
from std_msgs.msg import Float32MultiArray
from timeit import default_timer as timer

######Global Variables####

Angles     = numpy.arange(0,2,dtype = numpy.float)
Velocities = numpy.arange(0,2,dtype = numpy.float)
Torques    = numpy.arange(0,2,dtype = numpy.float)

############# Robot Parameters ##############
mhip     = 1100   #grams
mantom   = 178    #grams
lhip     = 245    #mm
lantom   = 194    #mm
lcghip   = 28     #mm
lcgantom = 78     #mm
Ixxhip   = 333468 #gmm2
Ixxantom = 898978 #gmm2

hnc = float(-1 * mantom * lhip * lcgantom)

G1Const1 = (mhip * lcghip + mantom * lhip) * 9.81
G1Const2 = (mantom * lcgantom * 9.81)

G2Const  = (mantom * lcgantom)
##############################################

pblshd = Float32MultiArray()

######Topics ISR##########

def CurrentState(data):
    Array = numpy.array(data.data)
    global Angles
    global Velocities
    global Torques
    Angles = Array[1:3]
    Angles[0] = math.pi - Angles[0]
    Angles[1] = -1 * Angles[1]
    Velocities = -1 * Array[25:27]
    Torques    = -1 * (1000) * (1000) * (1000) * numpy.round(Array[13:15],2) # recheck that    

######Main################
if __name__ == '__main__':
    
    # Nodes and Topics Initialisations 
    rospy.init_node('DMIntegralP', anonymous=False)
    rospy.Subscriber('getter', Float32MultiArray , CurrentState)
    pub = rospy.Publisher('IntegralPart', Float32MultiArray, queue_size = 2)
    rate = rospy.Rate(100)
   
    while not rospy.is_shutdown():
        start = timer()

        # Cristofel Matrix 
        h = hnc * math.sin(Angles[1])

        CT11 =      h *  Velocities[1]
        CT12 = -1 * h *  Velocities[0]
        CT21 =      h * (Velocities[0] + Velocities[1])
    
        CT = torch.Tensor([[CT11 , CT12],[CT21 , 0]]).cuda()
        #print("Cristofel")
        #print(CT)
        # Velocity Vector
        VelocityVec = torch.Tensor([[Velocities[0]],[Velocities[1]]]).cuda()
        #print("Velocities")
        #print(VelocityVec)
        # Motor Torque Vector
        TorqueMotorVec = torch.Tensor([[Torques[0]],[Torques[1]]]).cuda()
        #print("Torques")
        #print(TorqueMotorVec)
        # Gravity Vector
        temp = math.cos(Angles[0] + Angles [1])
        G1 = G1Const1 * math.cos(Angles[0]) + G1Const2 * temp
        G2 = G2Const * temp
        g = torch.Tensor([[G1],[G2]]).cuda()
        #print("G:")
        #print(g)
        # Integration
        IntegPart = TorqueMotorVec + torch.mm(CT,VelocityVec).cuda() - g

        pblshd.data = [IntegPart[0],IntegPart[1]]
        pub.publish(pblshd)
        
        end = timer()
        #print(end-start)
        rate.sleep()