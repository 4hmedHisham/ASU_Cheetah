#!/usr/bin/env python3
import rospy
import time
import GP2_Vrep_V3 as v
import numpy as np
from std_msgs.msg import Float32MultiArray
v.vrepInterface(19998)
time.sleep(0.5)
v.get_angles_firsttime()
def get_all_angels():

	pub = rospy.Publisher('vrep_angles', Float32MultiArray, queue_size=10)
	rospy.init_node('Node1', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		i=0
		total = Float32MultiArray()
		total.data = []
		vrepang=[]
		while i in range(12):
			angs=v.get_angles(i)
			#print(angs)
			vrepang.append(v.get_angles(i))
			i=i+1

		#print(vrepang)
		#rospy.loginfo(vrepang[0])
		print(vrepang)
		total.data=vrepang
		pub.publish(total)
		rate.sleep()
if __name__ == '__main__':
	try:
		
		get_all_angels()
	except rospy.ROSInterruptException:
		pass
