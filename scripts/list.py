#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

def callback(data):
	array = data.data
	array = np.reshape(array,(6,3))
	print(array)


def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('jac',Float32MultiArray,callback)
	rospy.spin()


if __name__ == '__main__':
	listener()
