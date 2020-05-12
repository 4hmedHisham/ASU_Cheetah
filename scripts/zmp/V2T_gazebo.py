#!/usr/bin/env python3
import rospy
import time
from timeit import default_timer  as timer
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
import gp2_gazebo as v

boolean = 1
mode = 'p'
#ADDED VREP Iinit in one function
counter=0
testing=False
printing=False

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
	global mode
	''' This function instialze the node responsible for vrep/ros interaction.'''
	pub = rospy.Publisher('getter', Float32MultiArray, queue_size=10)
	sub = rospy.Subscriber('setter',String,set_vrep_angels)
	#sub = rospy.Subscriber('disable',Bool,disable)
	rospy.Subscriber('torques',String,set_vrep_torques)
	rospy.init_node('vrep', anonymous=True)
	v.ros_init_gaz()	
	time.sleep(0.5)
	v.init_angles()
	time.sleep(0.5)	
	print("ROS NODE INTIALIZED")
	rate = rospy.Rate(1000) # 10hz	
	counterrr = 0
	init=0
	while not rospy.is_shutdown():
		start = timer()
		if init==0:
			#raw_input("INIT UR NODES")
			init=init+1
		
		# if counterrr == 1:
		# 		mode=raw_input("Please enter 'p' for position control or 't' for torque control:  ")
		# 		while ((mode!='p') and (mode!='t')):
		# 			print('Please enter a vaild control mode')
		# 			mode=raw_input("Please enter 'p' for position control or 't' for torque control:  ")
		# 		# if mode == 't':
		# 		# 	raw_input("Press enter any key to disable control loop: ")
    	# 		# 	v.ctrl_en(mode)
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
			# for i in range(12):
			# 	vrep_param.append(v.get_torque(i))
			# for i in range(12):
			# 	vrep_param.append(v.get_vel(i))
			# linear_accs=v.imu_read()
			# anglular_accs=v.gyro_read()
			# for linear_acc in linear_accs:
			# 	vrep_param.append(linear_acc)
			# for angular_acc in anglular_accs :
			# 	vrep_param.append(angular_acc)			
		else :
			while i in range(12):
				#print(angs)
				vrep_param.append(counter)
				counter=counter+1
				i=i+1

		total.data=vrep_param
		#print(total.data)
		pub.publish(total)
		counterrr = counterrr +1 
		#print(counterrr,"ahoo")
		#end = timer()
		#print("before sleep = ")
		#print(end - start)
		rate.sleep()
		end = timer()
		#print("After sleep = ")
		#print(end - start)

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
	#v.set_torque(joint,angle)
	if printing == True:
		print('Recieved ')
	# print(ang)


#SO GIT YA5OD BALO
if __name__ == '__main__':
	try:		
		start_vrep_node()
	except rospy.ROSInterruptException:
		pass

