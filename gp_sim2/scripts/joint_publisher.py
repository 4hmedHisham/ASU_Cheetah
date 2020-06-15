#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

import sys
import math
import numpy as np
import time

plus2pi=False

def angles():#Gets angles for leg trajectory  xc stands for current x
#tehta1=-133 theta2=+97
#    if init=='y':
#        xc=0
#    else:
#        xc=l1*np.cos(hipangle) + l2*np.cos(hipangle+kneeangle)#TEST
    
    s = 150 #stride lenth
    H = 100 #height
    T = 0.1 #total time for each cycle
    h = 100 #hsmall
    intial_leg_height = 524 #from ground to joint
    stp=100 #number of steps even number for some reason
    St=T/stp #sample time
    xnew = np.zeros([stp,1], dtype=float)
    t=0;
    i=0;
    xc=-62.56
    l1=300
    l2=300
    for t in np.arange(0,T,St):
         xnew[i]=(s*((t/T)-((1/(2*np.pi))*np.sin(2*np.pi*(t/T))))-(s/2)+s/2)+xc
         i=i+1;
    
    
    tnew = np.zeros([stp,1], dtype=float)
    i=0;
    
    # malhash lazma,ba2a leha lazma
    for t in np.arange(0,stp,1):
         tnew[i]=St*t
         i=i+1

    
    i=0
    ynew=np.zeros([stp,1], dtype=float)
     #First part of Ynew in peicewise
     
    for t in np.arange(0,T/2, St):
         ynew[i]=(-(h/(2*np.pi))*np.sin(((4*np.pi)/T)*t)+((2*h*t)/T)-(h/2))+(h/2)-intial_leg_height
         i=i+1;
    
    n=(T/2)
    for t in np.arange(n,T, St):
        ynew[i]= (-(h/(2*np.pi))*np.sin(4*np.pi-(((4*np.pi)/T)*t))-((2*h*t)/T)+((3*h)/2))+(h/2)-intial_leg_height
        i=i+1
        
    
       
    

    
    theta1=np.zeros([stp,1], dtype=float)
    theta2=np.zeros([stp,1], dtype=float)
   # plt.figure()
     
    i=0;
    for t in np.arange(0,T, St):
        xh=xnew[i]
        yh=ynew[i]
        r=np.sqrt(xh**2+yh**2)
        theta2[i]=np.arccos((r**2-l1**2-l2**2)/(2*l1*l2))
        if theta2[i]<0 and plus2pi==True:
            theta2[i]=theta2[i]+2*np.pi
        theta1[i]=math.atan2(yh,xh)-np.arcsin((l2*np.sin(theta2[i]))/r)
        if theta1[i]<0 and plus2pi==True:
            theta1[i]=theta1[i]+2*np.pi
        i=i+1
        
    
#    plt.plot(tnew ,ynew)
   # plt.plot(xnew,ynew)
#    plt.plot(tnew,theta1)
#    plt.plot(tnew,theta2)
   # plt.xlabel('Time')
   # plt.ylabel('y or x')
   # plt.show()
       
    return theta1,theta2


class JointPub(object):
    def __init__(self):

        self.publishers_array = []
      #  self._ab1_joint_pub = rospy.Publisher('/cheetah/ab1_joint_position_controller/command', Float64, queue_size=1)
    #    self._ab2_joint_pub = rospy.Publisher('/cheetah/ab2_joint_position_controller/command', Float64, queue_size=1)
     #   self._ab3_joint_pub = rospy.Publisher('/cheetah/ab3_joint_position_controller/command', Float64, queue_size=1)
     #   self._ab4_joint_pub = rospy.Publisher('/cheetah/ab4_joint_position_controller/command', Float64, queue_size=1)

        self._bc1_joint_pub = rospy.Publisher('/cheetah/bc1_joint_position_controller/command', Float64, queue_size=1)
        self._bc2_joint_pub = rospy.Publisher('/cheetah/bc2_joint_position_controller/command', Float64, queue_size=1)
        self._bc3_joint_pub = rospy.Publisher('/cheetah/bc3_joint_position_controller/command', Float64, queue_size=1)
        self._bc4_joint_pub = rospy.Publisher('/cheetah/bc4_joint_position_controller/command', Float64, queue_size=1)

        self._cd1_joint_pub = rospy.Publisher('/cheetah/cd1_joint_position_controller/command', Float64, queue_size=1)
        self._cd2_joint_pub = rospy.Publisher('/cheetah/cd2_joint_position_controller/command', Float64, queue_size=1)
        self._cd3_joint_pub = rospy.Publisher('/cheetah/cd3_joint_position_controller/command', Float64, queue_size=1)
        self._cd4_joint_pub = rospy.Publisher('/cheetah/cd4_joint_position_controller/command', Float64, queue_size=1)
     
    #    self.publishers_array.append(self._ab1_joint_pub)
    #    self.publishers_array.append(self._ab2_joint_pub)
     #   self.publishers_array.append(self._ab3_joint_pub)
     #   self.publishers_array.append(self._ab4_joint_pub)
  

        self.publishers_array.append(self._bc1_joint_pub)
        self.publishers_array.append(self._bc2_joint_pub)
        self.publishers_array.append(self._bc3_joint_pub)
        self.publishers_array.append(self._bc4_joint_pub)
       
        self.publishers_array.append(self._cd1_joint_pub)
        self.publishers_array.append(self._cd2_joint_pub)
        self.publishers_array.append(self._cd3_joint_pub)
        self.publishers_array.append(self._cd4_joint_pub)




    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.loginfo(str(joint_value))
          publisher_object.publish(joint_value)
          i += 1


    def start_loop(self, rate_value = 2.0):

	stp=100 #number of steps even number for some reason
        theta1=np.zeros([stp,1], dtype=float)
    	theta2=np.zeros([stp,1], dtype=float)
	delay=0.0083
	delay2=0.5
	delay3=delay*10
	tryy=7
        rospy.loginfo("Start Loop")
        theta1, theta2 = angles() 

        rate = rospy.Rate(rate_value)




        while not rospy.is_shutdown():

    	  
	# pos=[theta1[0],theta1[0],theta1[0],theta1[0],theta2[0],theta2[0],-theta2[0],theta2[0]]
	 # self.move_joints(pos)

	  #time.sleep(delay2)

	  for i in range(theta1.shape[0]):
	    pos=[theta1[0],theta1[0],theta1[i],theta1[0],theta2[0],theta2[0],-theta2[i],theta2[0]]
	    self.move_joints(pos)
            time.sleep(delay)

	  time.sleep(delay2)
          i=0

	  for i in range(theta1.shape[0]):
	    pos=[theta1[0],theta1[i],theta1[stp-1],theta1[0],theta2[0],theta2[i],-theta2[stp-1],theta2[0]]
	    self.move_joints(pos)
            time.sleep(delay)

	  time.sleep(delay2)
	  i=0





	  for i in range(theta1.shape[0]):
	    pos=[theta1[0],theta1[stp-1],theta1[stp-1],theta1[i],theta2[0],theta2[stp-1],-theta2[stp-1],theta2[i]]
	    self.move_joints(pos)
            time.sleep(delay)

	  time.sleep(delay2)

	  i=0




	  for i in range(theta1.shape[0]):
	    pos=[theta1[i],theta1[stp-1],theta1[stp-1],theta1[stp-1],theta2[i],theta2[stp-1],-theta2[stp-1],theta2[stp-1]]
	    self.move_joints(pos)
            time.sleep(delay)

	  time.sleep(delay2)
	  i=0








    	  rate.sleep()

if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 4.0
    joint_publisher.start_loop(rate_value)
