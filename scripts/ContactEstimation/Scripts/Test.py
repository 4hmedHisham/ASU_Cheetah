#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy

########################################

def talker():
    pub = rospy.Publisher('getter', Float32MultiArray, queue_size = 10) # defining Topic
    rospy.init_node('Test', anonymous = False) # defining node 
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
       	Result = Float32MultiArray()
        Array = numpy.arange(1,37,1)
        Result.data = Array
        pub.publish(Result)
        rate.sleep()       

########################################
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
