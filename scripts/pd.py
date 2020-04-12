#! /usr/bin/env python
import numpy as np
from numpy import sin , cos , arctan2
from timeit import default_timer as time
import rospy 
from std_msgs.msg import Float32MultiArray , String, Bool

start = time()
rospy.init_node('Impedance',anonymous=True)
pub= rospy.Publisher('torques',String,queue_size=10)
pub1 = rospy.Publisher('disable',Bool,queue_size=10)
pub2 = rospy.Publisher('t',Float32MultiArray,queue_size=10)
rate = rospy.Rate(10)

l1 = 244.59
l2 = 208.4
a = 20
kp = 0.5
kd = 0.01
sizeof_fwdk_array = 24 # x1 y1 z1 x2 y2 z2....
sizeof_desired_array = 3
sizeof_getter_array = 42
leg_no = 1
current_time = 0
prev_time = 0
x_current = 0
y_current = 0
x_desired = -390
y_desired = 0
prev_r = 0
prev_theta = 0
theta_knee = 0
trqs = Float32MultiArray()

def polar_jacoian(theta3):
    r_row = [[0, (-2 * l1 * l2 * sin(theta3)) / (2 * np.sqrt(l1**2 + l2**2 + (2 * l1 * l2 * cos(theta3))))]]
    theta_row = [[1, ((l2**2 + (l1*l2*cos(theta3))) / (l1**2 + l2**2 + (2 * l1 * l2 * cos(theta3))))]]
    Jp = np.concatenate((r_row, theta_row), axis=0)
    Jp = Jp.reshape(2,2)
    #print(theta3)
    return np.transpose(Jp)

def cart2polar(x,y):
    r = np.sqrt(x**2 + y**2)
    theta = arctan2(y, x)
    theta = theta * (180.0/np.pi)
    return r, theta


def current_pos(data):
    msg = data.data
    msg = np.reshape(msg,sizeof_fwdk_array)
    x = msg[(3*leg_no)-3] #[0 1 2 3 4 5 6 7 8 9 10 11]
    y = msg[(3*leg_no)-1]
    global x_current
    global y_current
    x_current = x
    y_current = y
    
def desired_pos(data):
    msg = data.data
    msg = np.reshape(msg,sizeof_desired_array)
    global x_desired
    global y_desired
    global leg_no
    x_desired = msg[0]
    y_desired = msg[1]
    leg_no = msg[2]

def current_theta(data):
    msg = data.data
    msg = np.reshape(msg,sizeof_getter_array)
    global theta_knee
    theta_knee = msg[(leg_no*3)-1] * (180.0/np.pi)


def pd():
    global current_time,prev_time
    global x_desired, y_desired,x_current,y_current
    global prev_r, prev_theta
    current_time = time()
    elapsed_time = current_time - prev_time
    error_x = x_desired - x_current #5
    error_y = y_desired - y_current #5
    r,theta = cart2polar(error_x,error_y)
    p_error = kp*(r+theta)
    d_error = kd*((r - prev_r)/elapsed_time) + kd*((theta - prev_theta)/elapsed_time)
    #print('r',r,prev_r,'-------','theta',theta,prev_theta)
    #print(error_x,error_y)
    prev_r = r
    prev_theta = theta
    prev_time = current_time
    output = [[p_error,d_error]]
    output = np.reshape(output,(2,1))
    #print(output)
    
    return output 
    
    
def listener_current_pos():
    rospy.Subscriber('fwd',Float32MultiArray,current_pos)

def listener_desired_pos():
    rospy.Subscriber('desired',Float32MultiArray,desired_pos)

def listener_theta():
    rospy.Subscriber('getter',Float32MultiArray,current_theta)


listener_current_pos()
listener_desired_pos()
t = Bool()
t = True
pub1.publish(t)
print("Subcribed")
while not rospy.is_shutdown():
    listener_theta()
    
    torques = np.matmul(polar_jacoian(theta_knee),pd())
    torques = torques.flatten()

    torques = np.clip(torques,-18,18)
    trqs.data = torques
    t = "%s %s"%((3*leg_no)-2,torques[0])
    #print(t)
    pub.publish(t)
    t = "%s %s"%((3*leg_no)-1 ,torques[1])
    pub.publish(t)
    pub2.publish(trqs)
    #print(np.matmul(pd(),polar_jacoian(theta_knee)))
    #print(start)
    #print(x_current,y_current)
    print(theta_knee)
    rate.sleep()
    