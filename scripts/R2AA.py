#!/usr/bin/env python3
import rospy
import time
import GP2_Vrep_V3 as v
import numpy as np
import threading
from std_msgs.msg import Float32MultiArray
testing=True
angles=0
if not testing :
	v.vrepInterface(19998)


def callback(data):
	#loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	#print('info is'+data.data)
	global angles
	angles=data.data


def initt():
	rospy.Subscriber('vrep_angles',Float32MultiArray,callback)
	rospy.init_node('GetAngFromTopic', anonymous=True)
	
def return_ang():
	global angles
	return 8


initt()

while(1):
	
	time.sleep(2)
	print(angles)

	

