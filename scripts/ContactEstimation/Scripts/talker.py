#!/usr/bin/env python

import rospy
import std_msgs.msg
import numpy
from ContactEstimation.msg import MatrixTwoxTwo

########################################

def talker():
    pub = rospy.Publisher('MassM', MatrixTwoxTwo, queue_size=10) # defining Topic Called mass matrix
    rospy.init_node('MassMartixNode', anonymous=True) # defining node 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       	Result = MatrixTwoxTwo()
        Result.R1C1 = 2
        Result.R1C2 = 1
        Result.R2C1 = 1
        Result.R2C2 = 2
        pub.publish(Result)
        rate.sleep()       

########################################
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
