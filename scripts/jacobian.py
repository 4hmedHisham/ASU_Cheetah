#! /usr/bin/env python
import numpy as np
from numpy import sin , cos
from timeit import default_timer as timer
import rospy 
from std_msgs.msg import Float32MultiArray


rospy.init_node('jacobian',anonymous=True)
pub = rospy.Publisher('jac',Float32MultiArray,queue_size=10)
#sub = rospy.Subscriber('getter',Float32MultiArray,Jacobian_3D)
import numpy as np
from numpy import sin, cos , arctan2
from timeit import default_timer as timer

l1 = 244.59
l2 = 208.4
a = 20
start = timer()


def Jacobian_3D(theta1, theta2, theta3):
    '''
    Jv2 = [[0,0,0],[0,0,0],[0,0,0]]
    Jw2 = [[0,0,1],[0,0,0],[0,0,0]]
    Jv3 = [[l1*sin(theta1)*sin(theta2)+a*cos(theta1),l2*cos(theta2),0],
           [-l1*sin(theta2)+2*a*sin(theta1)*cos(theta1),l2*sin(theta1)*cos(theta2),l2*cos(theta1)*cos(theta2)],[0,0,0]]
    Jw3 = [[0,0,1],[0,-cos(theta1),-sin(theta1)],[0,0,0]]
    '''
    Jv4 = [[l2*sin(theta1)*sin(theta2+theta3)+l1*sin(theta1)*sin(theta2)-a*cos(theta1)
               ,l2*cos(theta2+theta3)+l1*cos(theta1),0],
           [-2*l2*sin(theta1)*cos(theta1)*sin(theta2 + theta3)-2*l1*cos(theta1)*sin(theta1)*sin(theta2)
               +a*(cos(theta1))**2 -a*(sin(theta1))**2 ,-l2*cos(theta1)*cos(theta2 + theta3)-l1*cos(theta1)*cos(theta2)
               ,l2*sin(theta1)*cos(theta2+theta3)+l1*sin(theta1)+cos(theta2)]
        ,[-l2*sin(theta2+theta3),-l2*sin(theta1)*cos(theta2+theta3),l2*cos(theta1)*cos(theta2+theta3)]]
    Jw4 = [[0, 0, 1], [0, -cos(theta1), -sin(theta1)], [0, -cos(theta1), -sin(theta1)]]

    J = np.concatenate((Jv4, Jw4), axis=0)
    return J


def polar_jacoian(theta3):
    r_row = [[0, -2 * l1 * l2 * sin(theta3) * 1 / (2 * np.sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * cos(theta3)))]]
    theta_row = [[1, (l2 * l2 + l1 * l2 * cos(theta3) / (l1 * l1 + l2 * l2 + 2 * l1 * l2 * cos(theta3)))]]
    Jp = np.concatenate((r_row, theta_row), axis=0)
    return Jp


def cart2polar(x, y):
    r = np.sqrt(x * x + y * y)
    theta = arctan2(y, x)
    return r, theta


print(Jacobian_3D(10,20,30))
end = timer()
print('Exec time:')
print((end - start)*1000,'ms')






