#!/usr/bin/env python

##############################################

import rospy
from std_msgs.msg import String
import numpy
from ContactEstimation.msg import MatrixTwoxTwo
from timeit import default_timer as timer
import torch

##############################################

JustTest = numpy.zeros((2,2))
Result = torch.ones(2,2).cuda()
FastJustTest = torch.Tensor([[2, 1],[4, 5]]).cuda()
end = 0.00297
start = 0.00297
Tokens = numpy.zeros(2)
count_1 = 0
count_2 = 0
Var = torch.ones(2,2).cuda()
##############################################

def ReadTopic(data):
    JustTest = numpy.zeros((2,2))
    JustTest[0,0] = data.R1C1
    JustTest[0,1] = data.R1C2
    JustTest[1,0] = data.R2C1
    JustTest[1,1] = data.R2C2
    return JustTest	

##############################################

def PubIfUpdated():
    global Result

    if (Tokens[0] == 1 and Tokens[1] == 1 and count_1 ==count_2):
        Tokens[0] = 0
        Tokens[1] = 0
        start = timer()
        Result = torch.mm(FastJustTest,Var).cuda()
        end = timer()
        print(Result)
        print('////////////////\n')
        print(end-start)
        print('////////////////\n')

##############################################

def MassMat(data):
    global count_1
    global FastJustTest
    global Tokens
    count_1 = count_1 + 1
    JustTest = ReadTopic(data)
    FastJustTest = torch.from_numpy(JustTest).cuda()
    Tokens[0] = 1
    PubIfUpdated()

###############################################

def ChristoffelMat(data):
	#print('\nChristoffel:')
    global count_2
    global Var
    global Tokens

    Var = torch.from_numpy(ReadTopic(data)).cuda()
    count_2 = count_2 + 1	
    Tokens[1] = 1
    PubIfUpdated()

###############################################

def listener():
	rospy.init_node('DynamicModel', anonymous=False)
	rospy.Subscriber('MassM', MatrixTwoxTwo , MassMat)
	rospy.Subscriber('ChristoffelM', MatrixTwoxTwo , ChristoffelMat)
	rospy.spin()

################################################
if __name__ == '__main__':
    listener()
