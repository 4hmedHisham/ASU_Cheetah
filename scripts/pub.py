#! /usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
	pub= rospy.Publisher('pos',Float32MultiArray,queue_size=10)
	rospy.init_node('talker',anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		msg = Float32MultiArray()
		msg.data = [10,12]
		pub.publish(msg)
		rate.sleep()

while not rospy.is_shutdown():
	talker()

