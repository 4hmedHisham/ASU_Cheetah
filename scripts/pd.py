#! /usr/bin/env python
import numpy as np
from numpy import sin , cos , arctan2
from timeit import default_timer as time
import rospy 
from std_msgs.msg import Float32MultiArray

start = time()
rospy.init_node('pd',anonymous=True)
rate = rospy.Rate(10)

l1 = 244.59
l2 = 208.4
a = 20
kp = 1.0
kd = 0.1
sizeof_fwdk_array = 2 # x1 y1 z1 x2 y2 z2....
leg_no = 1
xc = 0
yc = 0


def polar_jacoian(theta3):
    r_row = [[0, -2 * l1 * l2 * sin(theta3) * 1 / (2 * np.sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * cos(theta3)))]]
    theta_row = [[1, (l2 * l2 + l1 * l2 * cos(theta3) / (l1 * l1 + l2 * l2 + 2 * l1 * l2 * cos(theta3)))]]
    Jp = np.concatenate((r_row, theta_row), axis=0)
    return Jp


def cart2polar(data):
    msg = data.data
    msg = np.reshape(msg,2)
    x = msg[0]
    y = msg[1]
    r = np.sqrt(x * x + y * y)
    theta = arctan2(y, x)
    print(r,theta)
    #return r, theta

def current_pos(data):
    msg = data.data
    msg = np.reshape(msg,sizeof_fwdk_array)
    xc = msg[0] #[0 1 2 3 4 5 6 7 8 9 10 11]
    yc = msg[1]
    return xc, yc
    



def listener():
    sub = rospy.Subscriber('getter',Float32MultiArray,current_pos)
    return sub.callback.func_globals

while(1):
    x,y = listener()
    rate.sleep()
   


'''
def listener():
	rospy.Subscriber('pos',Float32MultiArray,cart2polar)

#print(Jacobian_3D(10,20,30))
while not rospy.is_shutdown():
    start = time()
    Jacobian_3D(10,20,50)
    listener()
    end = time()
    print(end-start)




#print('Exec time:')
#print((end - start)*1000,'ms')

'''