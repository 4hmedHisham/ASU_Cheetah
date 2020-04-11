import rospy

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

pub = rospy.Publisher('torques', String, queue_size=10)	
rospy.init_node('tstr', anonymous=True)
print("ROS NODE INTIALIZED")


#pub.publish("1 3")

pub.publish("2 5")