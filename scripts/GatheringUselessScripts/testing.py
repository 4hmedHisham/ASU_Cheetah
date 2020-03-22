#!/usr/bin/env python3
import rospy
import time
import GP2_Vrep_V3 as v
import numpy as np
from std_msgs.msg import Float32MultiArray
angles=0
class angle :

	def __init__(self):  
        	self.angles = 0
		self.sub=rospy.Subscriber("vrep_angles", Float32MultiArray, self.callback)  
	def callback(self,data):
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
		#print('sup')
		self.angles=data.data
		print('angels inner')
		print(self.angles)#This Works
	def get_angles(self):
		print('IM IN')
		return self.angles
		

if __name__ == '__main__':
	try:
		rospy.init_node('angelsToAlg', anonymous=True)
		guy=angle()
		print('angels outer')
		print(guy.get_angles())
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
