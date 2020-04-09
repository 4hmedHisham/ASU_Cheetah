#!/usr/bin/env python3
import rospy
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool

boolean = 1
mode = 'p'

pub = rospy.Publisher('disable', Bool)
rospy.init_node('TESTAAAAAA3', anonymous=True)

pub.publish(0)
print("NO")
pub.publish(1)