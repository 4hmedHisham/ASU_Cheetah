#!/usr/bin/env python

######imports#############

import rospy
import numpy
import torch
import math
from std_msgs.msg import Float32MultiArray
from timeit import default_timer as timer


#Coventions for getter
# intial_name = ['ab3', 'bc3', 'cd3', 'ab4', 'bc4', 'cd4', 'ab1', 'bc1', 'cd1', 'ab2', 'bc2', 'cd2']
# angles - torques - velocities
# rad - N m - rad/s


######Global Variables####

Angles     = numpy.arange(0,2,dtype = numpy.float)
Velocities = numpy.arange(0,2,dtype = numpy.float)

############# Robot Parameters ##############
mhip     = 1100   #grams
mantom   = 178    #grams
lhip     = 245    #mm
lantom   = 194    #mm
lcghip   = 28     #mm
lcgantom = 78     #mm
Ixxhip   = 333468 #gmm2
Ixxantom = 898978 #gmm2

M11_PConst = float(mhip * math.pow(lcghip,2) + mantom * ( math.pow(lhip,2) + math.pow(lcgantom,2) + 2 * lhip * math.pow(lcgantom,2) ) + Ixxantom + Ixxhip)
M11_MConst = float(2 * lhip * lcgantom * mantom)

M12_PConst = float(mantom * (math.pow(lcgantom,2)) + Ixxantom )
M12_MConst = float(mantom * lhip * lcgantom)

M22        = float(mantom * (math.pow(lcgantom,2)) + Ixxantom)
###############################################
pblshd = Float32MultiArray()

######Topics ISR##########

def CurrentState(data):
    Array = numpy.array(data.data)
    global Angles
    Angles = Array[1:3]
    Angles[0] = math.pi - Angles[0]
    Angles[1] = -1 * Angles[1]
    global Velocities
    Velocities = -1 * Array[25:27] #book's +ve direction of rotataion mapping

######Main################
if __name__ == '__main__':
    rospy.init_node('GeneralizedMomentum', anonymous=False)
    rospy.Subscriber('getter', Float32MultiArray , CurrentState)
    pub = rospy.Publisher('GM', Float32MultiArray, queue_size = 2)
    rate = rospy.Rate(100)
   
    while not rospy.is_shutdown():
        start = timer()
        
        x = math.cos(Angles[1])
        
        M11 = M11_PConst + M11_MConst * x
        M12 = M12_PConst + M12_MConst * x

        MassMatrix = torch.Tensor([[M11 , M12],[M12 , M22]]).cuda()
        VelocityVec = torch.Tensor([[Velocities[0]],[Velocities[1]]]).cuda()
        #print("Velocity")
        #print(VelocityVec)
        #print("Mass Matrix")
        #print(MassMatrix)
        GeneralizedMomentum = torch.mm(MassMatrix,VelocityVec).cuda()
        
        pblshd.data = [GeneralizedMomentum[0],GeneralizedMomentum[1]]
        pub.publish(pblshd)

        #end = timer()
        #print(end-start)
        rate.sleep()
