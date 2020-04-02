#!/usr/bin/env python

import rospy
import std_msgs.msg
import numpy
from ContactEstimation.msg import MatrixTwoxTwo

########################################

def talker():
    pub = rospy.Publisher('ChristoffelM', MatrixTwoxTwo, queue_size=10)
    rospy.init_node('ChristoffelMartixNode', anonymous=False) # defining node 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       	Result = MatrixTwoxTwo()
        Result.R1C1 = 3
        Result.R1C2 = 4
        Result.R2C1 = 5
        Result.R2C2 = 10
        pub.publish(Result)
        rate.sleep()       

########################################
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
