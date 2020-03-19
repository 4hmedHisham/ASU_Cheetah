import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
class ros:
    def __init__(self,pub_or_sub,Node_Name,topics,func=None):
        if(pub_or_sub=='p'):
            
            self.pub = rospy.Publisher(topics,Float32MultiArray, queue_size=10)
        elif(pub_or_sub=='s'):
            
            self.sub=rospy.Subscriber(topics,Float32MultiArray,func)
        elif(pub_or_sub=='b'):
            print('IN IF')
            self.pub = rospy.Publisher(topics[0], Float32MultiArray, queue_size=10)
            rospy.Subscriber(topics[1],Float32MultiArray,func) 
        else:
            print('No Valid Argument Passed')
            #return 0
        print('AFTER IF')
        time.sleep(2)
        rospy.init_node(Node_Name, anonymous=True)
        time.sleep(2)
        
        print('Started')
        #pub.publish([1,2,3])
    def ros_publish(self,msg):
        data=Float32MultiArray()
        data.data=msg
        self.pub.publish(data)



        

