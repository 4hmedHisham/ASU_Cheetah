#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

import sys
import math
import numpy as np
import time


 
class JointPub(object):
    def __init__(self):

        self.publishers_array = []
        self._ab1_pub = rospy.Publisher('/SimplifiedAssembly/ab1_position_controller/command', Float64, queue_size=1)
        self._bc1_pub = rospy.Publisher('/SimplifiedAssembly/bc1_position_controller/command', Float64, queue_size=1)
        self._cd1_pub = rospy.Publisher('/SimplifiedAssembly/cd1_position_controller/command', Float64, queue_size=1)
        self._ab2_pub = rospy.Publisher('/SimplifiedAssembly/ab2_position_controller/command', Float64, queue_size=1)
        self._bc2_pub = rospy.Publisher('/SimplifiedAssembly/bc2_position_controller/command', Float64, queue_size=1)
        self._cd2_pub = rospy.Publisher('/SimplifiedAssembly/cd2_position_controller/command', Float64, queue_size=1)
        self._ab3_pub = rospy.Publisher('/SimplifiedAssembly/ab3_position_controller/command', Float64, queue_size=1)
        self._bc3_pub = rospy.Publisher('/SimplifiedAssembly/bc3_position_controller/command', Float64, queue_size=1)
        self._cd3_pub = rospy.Publisher('/SimplifiedAssembly/cd3_position_controller/command', Float64, queue_size=1)
        self._ab4_pub = rospy.Publisher('/SimplifiedAssembly/ab4_position_controller/command', Float64, queue_size=1)
        self._bc4_pub = rospy.Publisher('/SimplifiedAssembly/bc4_position_controller/command', Float64, queue_size=1)
        self._cd4_pub = rospy.Publisher('/SimplifiedAssembly/cd4_position_controller/command', Float64, queue_size=1)
     
        self.publishers_array.append(self._ab1_pub)
        self.publishers_array.append(self._bc1_pub)      
        self.publishers_array.append(self._cd1_pub)
        self.publishers_array.append(self._ab2_pub)
        self.publishers_array.append(self._bc2_pub)
        self.publishers_array.append(self._cd2_pub)
        self.publishers_array.append(self._ab3_pub)
        self.publishers_array.append(self._bc3_pub)
        self.publishers_array.append(self._cd3_pub)        
        self.publishers_array.append(self._ab4_pub)        
        self.publishers_array.append(self._bc4_pub)
        self.publishers_array.append(self._cd4_pub)

    def move_joints(self, joints_array):
        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.loginfo(str(joint_value))
          publisher_object.publish(joint_value)
          i += 1

    def start_loop(self, rate_value = 2.0):

        while not rospy.is_shutdown():
#            pos=[0,1.57,0,0,1.57,0,0,1.57,0,0,1.57,0]
            pos=[0,1.57,0,0,-1.57,0,0,-1.57,0,0,1.57,0]
#            pos=[0,0,0,0,0,0,0,0,0,0,0,0]
            self.move_joints(pos)
            time.sleep(0.5)


if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 4.0
    joint_publisher.start_loop(rate_value)
