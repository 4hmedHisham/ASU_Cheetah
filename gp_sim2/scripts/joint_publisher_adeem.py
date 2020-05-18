#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

class JointPub(object):
    def __init__(self):

        self.publishers_array = []
        self._bc1_joint_pub = rospy.Publisher('/cheetah/bc1_joint_position_controller/command', Float64, queue_size=1)
        self._bc2_joint_pub = rospy.Publisher('/cheetah/bc2_joint_position_controller/command', Float64, queue_size=1)
        self._bc3_joint_pub = rospy.Publisher('/cheetah/bc3_joint_position_controller/command', Float64, queue_size=1)
        self._bc4_joint_pub = rospy.Publisher('/cheetah/bc4_joint_position_controller/command', Float64, queue_size=1)

        self._cd1_joint_pub = rospy.Publisher('/cheetah/cd1_joint_position_controller/command', Float64, queue_size=1)
        self._cd2_joint_pub = rospy.Publisher('/cheetah/cd2_joint_position_controller/command', Float64, queue_size=1)
        self._cd3_joint_pub = rospy.Publisher('/cheetah/cd3_joint_position_controller/command', Float64, queue_size=1)
        self._cd4_joint_pub = rospy.Publisher('/cheetah/cd4_joint_position_controller/command', Float64, queue_size=1)
       

        self.publishers_array.append(self._bc1_joint_pub)
        self.publishers_array.append(self._bc2_joint_pub)
        self.publishers_array.append(self._bc3_joint_pub)
        self.publishers_array.append(self._bc4_joint_pub)
       
        self.publishers_array.append(self._cd1_joint_pub)
        self.publishers_array.append(self._cd2_joint_pub)
        self.publishers_array.append(self._cd3_joint_pub)
        self.publishers_array.append(self._cd4_joint_pub)




    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.loginfo(str(joint_value))
          publisher_object.publish(joint_value)
          i += 1


    def start_loop(self, rate_value = 2.0):
        rospy.loginfo("Start Loop")
        pos1 = [-2.3,-2.3,-2,-2,1.57,1.57,-1.57,1.57]
        pos2 = [-2.3,-2.3,-2,-2,0.0,0.0,0.0,0.0]
        position = "pos1"
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
          if position == "pos1":
            self.move_joints(pos1)
            position = "pos2"
          else:
            self.move_joints(pos2)
            position = "pos1"
          rate.sleep()


if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 4.0
    joint_publisher.start_loop(rate_value)
