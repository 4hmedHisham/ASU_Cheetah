#!/usr/bin/env python3
import rospy
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import GP2_Vrep_V3 as v
#ADD THIS COMMENT HERE FOR TESTING PURPOSES
counter=0
testing=False
if not testing :
	v.vrepInterface(19999)
	time.sleep(0.5)
	v.get_angles_firsttime()
	v.get_torques_firsttime()
	time.sleep(3)
	print('Vrep Up and Running')
def set_vrep_angels(data):
	joint_and_ang=data.data
	print('data is '+joint_and_ang)
	joint=joint_and_ang[0:3]
	ang=float(joint_and_ang[3:])
	v.set_angle(joint,ang)
	print('Recieved ')
	# print(ang)
def start_vrep_node():
	
	pub = rospy.Publisher('getter', Float32MultiArray, queue_size=10)
	sub = rospy.Subscriber('setter',String,set_vrep_angels)	
	rospy.init_node('vrep', anonymous=True)
	rate = rospy.Rate(100) # 10hz
	while not rospy.is_shutdown():
		i=0
		total = Float32MultiArray()
		total.data = []
		vrep_param=[]
		global counter
		if not testing :
			for i in range(12):
				#angs=v.get_angles(i)
				#print(angs)
				vrep_param.append(v.get_angles(i))
			for i in range(12):
				vrep_param.append(v.get_torque(i))


				
		else :
			while i in range(12):
				#print(angs)
				vrep_param.append(counter)
				counter=counter+1
				i=i+1

		total.data=vrep_param
		pub.publish(total)
		rate.sleep()
if __name__ == '__main__':
	try:
		
		start_vrep_node()
	except rospy.ROSInterruptException:
		pass

