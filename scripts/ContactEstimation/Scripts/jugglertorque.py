import rospy
import numpy
import torch
import math
from std_msgs.msg import Float32MultiArray, Float32
from timeit import default_timer as timer


Torques  = numpy.arange(0,2,dtype = numpy.float)

######Topics ISR##########

def CurrentState(data):
    Array = numpy.array(data.data)
    global Torques
    Torques    = -1 * Array[13:15] # recheck that    
    pub_1.publish(Torques[0])
    pub_2.publish(Torques[1])

######Main################
if __name__ == '__main__':
    
    # Nodes and Topics Initialisations 
    rospy.init_node('Tm_Testing', anonymous=False)
    rospy.Subscriber('getter', Float32MultiArray , CurrentState)
    pub_1 = rospy.Publisher('Tm_hip', Float32, queue_size = 1)
    pub_2 = rospy.Publisher('Tm_Knee', Float32, queue_size = 1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()
        
