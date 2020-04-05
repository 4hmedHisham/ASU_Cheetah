#!/usr/bin/env python3
import rospy
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import GP2_Vrep_V3 as v
#ADDED VREP Iinit in one function
counter=0
testing=False
printing=False
if not testing :
	v.vrep_init(19999)
def is_number(n):
    try:
        float(n)   # Type-casting the string to `float`.
                   # If string is not a valid `float`, 
                   # it'll raise `ValueError` exception
    except ValueError:
        return False
    return True
def set_vrep_angels(data):
	''' This callback function takes angels from topic "getter" and feed it to vrep using python api.'''
	switch=False
	joint=[]
	ang=[]
	joint_and_ang=data.data
	if printing == True:
		print('data is '+joint_and_ang)
	for letter in joint_and_ang:
		if switch ==True:
			ang.append(letter)
		if (letter != ' ') and (switch == False) :
			joint.append(letter)
		else:
			switch=True
	ang=''.join(ang)
	if printing == True:
		print("ANGLE IS ")
		print(ang)
		print("Joint Is ")
		print("joint in strings should be ")
		print(''.join(joint))
	value=is_number(''.join(joint))
	if printing == True:
		print(value)
	if value:
		joint=int(''.join(joint))
		print(joint)
	else:
		joint=''.join(joint)
		print(joint)
	angle=float(ang)
	
	print(angle)
	v.set_angle(joint,angle)
	if printing == True:
		print('Recieved ')
	# print(ang)
def start_vrep_node():
	''' This function instialze the node responsible for vrep/ros interaction.'''
	pub = rospy.Publisher('getter', Float32MultiArray, queue_size=10)
	sub = rospy.Subscriber('setter',String,set_vrep_angels)
	rospy.Subscriber('torques',String,set_vrep_torques)	
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
			for i in range(12):
				vrep_param.append(v.get_vel(i))
			linear_accs=v.imu_read()
			anglular_accs=v.gyro_read()
			for linear_acc in linear_accs:
				vrep_param.append(linear_acc)
			for angular_acc in anglular_accs :
				vrep_param.append(angular_acc)
			


				
		else :
			while i in range(12):
				#print(angs)
				vrep_param.append(counter)
				counter=counter+1
				i=i+1

		total.data=vrep_param
		#print(total.data)
		pub.publish(total)

		rate.sleep()

def set_vrep_torques(data):
	'''This callback function feeds torques fround in topic "torques" to vrep though python api function'''
	switch=False
	joint=[]
	ang=[]
	joint_and_ang=data.data
	if printing == True:
		print('data is '+joint_and_ang)
	for letter in joint_and_ang:
		if switch ==True:
			ang.append(letter)
		if (letter != ' ') and (switch == False) :
			joint.append(letter)
		else:
			switch=True
	ang=''.join(ang)
	if printing == True:
		print("Torque IS ")
		print(ang)
		print("Joint Is ")
		print("joint in strings should be ")
		print(''.join(joint))
	value=is_number(''.join(joint))
	if printing == True:
		print(value)
	if value:
		joint=int(''.join(joint))
		print(joint)
	else:
		joint=''.join(joint)
		print(joint)
	angle=float(ang)
	
	print(angle)
	v.set_torque(joint,angle)
	if printing == True:
		print('Recieved ')
	# print(ang)

if __name__ == '__main__':
	try:
		
		start_vrep_node()
	except rospy.ROSInterruptException:
		pass

