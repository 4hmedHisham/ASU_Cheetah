#!/usr/bin/env python

import rospy
import numpy as np
import sympy as sp
import math
from numpy import sin , cos
import scipy.linalg
from sympy.solvers import solve
from sympy import Symbol, E
from timeit import default_timer as time
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from Main_Functions import GP2_Function_V7 as gait

# Constants
g = 9.81

hipOffset = 300.78
mass = 20
z = 0
var =0
var2 = 0
var3 =0
var4 = 0
delay_seq = 0.2
first_step_flag=1

global pub
global leg_pos3_hip
global leg_pos4_hip
global leg_pos1_hip
global leg_pos2_hip
global leg_pos3_cg
global leg_pos4_cg
global leg_pos1_cg
global leg_pos2_cg
global leg3_ang
global leg4_ang        
global leg1_ang
global leg2_ang

# Parameters:
l1 =245
l2 =208.4
a=gait.a
initial_leg_height = 390 
stride = 80
h = 30    # maximum height of trajectory
ct=0.2
cycle_time =ct  # total time for each cycle
steps = 20 # number of steps 'even'
initial_distance = 0 # along x

x_fixed = np.zeros([steps, 1], dtype=float)
y_fixed = np.zeros([steps, 1], dtype=float)



initial_leg_height_f = initial_leg_height
stride_f = 80
h_f = h
cycle_time_f = ct
initial_distance_f = None
sample_time_f = None
pub = 0
indx_fix = 0
linear_acc_threshold = 20 
angular_acc_threshold = 20


#initial position relative to hip and cg
leg1_initial_hip=np.zeros([3, 1], dtype=float)
leg2_initial_hip=np.zeros([3, 1], dtype=float)
leg3_initial_hip=np.zeros([3, 1], dtype=float)
leg4_initial_hip=np.zeros([3, 1], dtype=float)

leg1_initial_cg=np.zeros([3, 1], dtype=float)
leg2_initial_cg=np.zeros([3, 1], dtype=float)
leg3_initial_cg=np.zeros([3, 1], dtype=float)
leg4_initial_cg=np.zeros([3, 1], dtype=float)

#Subscribed data
lin_acc=np.zeros([3, 1], dtype=float)
Ang_acc=np.zeros([3, 1], dtype=float)
lin_acc_prev=np.zeros([3, 1], dtype=float)
Ang_acc_prev=np.zeros([3, 1], dtype=float)


leg_pos3_hip=np.zeros([3, 1], dtype=float) 
leg_pos4_hip=np.zeros([3, 1], dtype=float)
leg_pos1_hip=np.zeros([3, 1], dtype=float)
leg_pos2_hip=np.zeros([3, 1], dtype=float) 

leg_pos3_cg=np.zeros([3, 1], dtype=float) 
leg_pos4_cg=np.zeros([3, 1], dtype=float)
leg_pos1_cg=np.zeros([3, 1], dtype=float)
leg_pos2_cg=np.zeros([3, 1], dtype=float)

leg3_ang=np.zeros([3, 1], dtype=float)                  #trans hip knee
leg4_ang=np.zeros([3, 1], dtype=float)
leg1_ang=np.zeros([3, 1], dtype=float)
leg2_ang=np.zeros([3, 1], dtype=float)
legfix_Prev_angs = np.zeros([3, 1], dtype=float)
legvar_Prev_angs = np.zeros([3, 1], dtype=float)


########################################################################################################

def imudata(data):
    Array = np.array(data.data)
    global lin_acc
    global Ang_acc
    global lin_acc_prev
    global Ang_acc_prev
    global linear_acc_threshold
    global angular_acc_threshold
    global leg3_ang                   #trans hip knee
    global leg4_ang
    global leg1_ang
    global leg2_ang
    global z        

    
    # linear_acc_threshold = 10
    # angular_acc_threshold = 10
    
    lin_acc_prev=lin_acc    ######save previous imu readings
    Ang_acc_prev=Ang_acc
    
    lin_acc = Array[36:39]  ######get new readings
    Ang_acc = Array[39:42]

    leg3_ang = Array[0:3] 
    leg4_ang = Array[3:6]
    leg1_ang = Array[6:9]
    leg2_ang = Array[9:12]    
  
    # for i in range (3):   ###### thresholding  between previous and new readings
    #     if ((np.absolute(lin_acc[i]-lin_acc_prev[i]))>linear_acc_threshold) or ((np.absolute(Ang_acc[i]-Ang_acc_prev[i]))> angular_acc_threshold):
    #         z = 1 
    #     else:
    #         z = 0


########################################################################################################

def leg_pos(data):
    Array2 = np.array(data.data)

    global leg_pos3_hip
    global leg_pos4_hip
    global leg_pos1_hip
    global leg_pos2_hip
    global leg_pos3_cg
    global leg_pos4_cg
    global leg_pos1_cg
    global leg_pos2_cg

    leg_pos3_hip =Array2[0:3] 
    leg_pos4_hip =Array2[3:6] 
    leg_pos1_hip =Array2[6:9] 
    leg_pos2_hip =Array2[9:12] 

    leg_pos3__cg =Array2[12:15] 
    leg_pos4__cg =Array2[15:18] 
    leg_pos1__cg =Array2[18:21] 
    leg_pos2__cg =Array2[21:24]
	


##########################################################################################################

# Functions:

def Gate_Publisher(pair_no,st):

    # Parameters:
    global l1
    global l2
    global initial_leg_height
    global initial_leg_height_f
    global stride
    global stride_f
    global h
    global h_f
    global cycle_time
    global cycle_time_f
    global steps
    global initial_distance
    global initial_distance_f
    global z
    global sample_time_f

    global leg_pos3_hip
    global leg_pos4_hip
    global leg_pos1_hip
    global leg_pos2_hip

    global leg1_ang
    global leg2_ang
    global leg3_ang
    global leg4_ang

    global x_fixed
    global y_fixed
    global var 
    global var2
    global var3 
    global var4    
    global first_step_flag

    flag_start =0
    x_current = 0 
    y_current = 0
    x_current_f =0
    global indx_fix
    t_left = 0
    current = 0
    last_fix = 0
    sample_time = np.float(cycle_time) / steps # sample time
    sample_time_f =np.float(cycle_time_f) / steps   # sample time
    stride= st
    stride_f=st

    if pair_no==1 :    #odd number pair 
        
        initial_distance_f = leg_pos1_hip[0]
        initial_distance = leg_pos3_hip[0]
        legfix_Prev_angs = leg1_ang
        legvar_Prev_angs = leg3_ang
        y_offset_f=leg_pos1_hip[1]                          #leg1
        y_offset=leg_pos3_hip[1]                           #leg3
        y_plane_f=np.full((steps+1, 1), y_offset_f)
        y_plane=np.full((steps+1, 1), y_offset)
        L_f='l'
        L_v='r'        

        if  first_step_flag==1  :
                if initial_distance_f >=0:
                    stride_f_mod=stride_f-initial_distance_f
                else: 
                    stride_f_mod=stride_f+np.abs(initial_distance_f)

                if initial_distance >=0:
                    stride_mod=stride_f-initial_distance
                else: 
                    stride_mod=stride_f+np.abs(initial_distance)

                first_step_flag=first_step_flag+1
                

        else:
            error_f=stride_f/2 -np.abs(initial_distance_f)
            error=stride/2 -np.abs(initial_distance)

            if error_f >=0:
                    stride_f_mod=stride_f-error_f
            else: 
                    stride_f_mod=stride_f+np.abs(error_f)   

            if error >=0:
                    stride_mod=stride-error
            else: 
                    stride_mod=stride+np.abs(error)
           

        #leg no.1
        xnew = np.zeros([steps, 1], dtype=float)
        t = 0
        i = 0

        for t in np.arange(0, cycle_time_f, sample_time_f):
            xnew[i] = (stride_f_mod * ((t / cycle_time_f) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time_f)))) - (stride_f_mod / 2) + stride_f_mod / 2) + initial_distance_f
            i = i + 1

        i = 0
        ynew = np.zeros([steps, 1], dtype=float)

        # First part of Ynew in piecewise
        for t in np.arange(0, cycle_time_f / 2, sample_time_f):
            ynew[i] = (-(h_f / (2 * np.pi)) * np.sin(((4 * np.pi) / cycle_time_f) * t) + ((2 * h_f * t) / cycle_time_f) - (h_f / 2)) + (h_f / 2) - initial_leg_height_f
            i = i + 1
        # second part of Ynew in piecewise
        n = (cycle_time_f / 2)
        for t in np.arange(n, cycle_time_f, sample_time_f):
            ynew[i] = (-(h_f / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time_f) * t)) - ((2 * h_f * t) / cycle_time_f) + ((3 * h_f) / 2)) + (h_f / 2) - initial_leg_height_f
            i = i + 1

        x_fixed = xnew
        y_fixed = ynew

        #leg no.2
        i = 0
        t = 0
        xnew = np.zeros([steps, 1], dtype=float)

        for t in np.arange(0, cycle_time, sample_time):
            xnew[i] = (stride_mod * ((t / cycle_time) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time)))) - (stride_mod / 2) + stride_mod / 2) + initial_distance
            i = i + 1

        i = 0
        ynew = np.zeros([steps, 1], dtype=float)

        # First part of Ynew in piecewise
        for t in np.arange(0, cycle_time / 2, sample_time):
            ynew[i] = (-(h / (2 * np.pi)) * np.sin(((4 * np.pi) / cycle_time) * t) + ((2 * h * t) / cycle_time) - (h / 2)) + (h / 2) - initial_leg_height
            i = i + 1
        # second part of Ynew in piecewise
        n = (cycle_time / 2)
        for t in np.arange(n, cycle_time, sample_time):
            ynew[i] = (-(h / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time) * t)) - ((2 * h * t) / cycle_time) + ((3 * h) / 2)) + (h / 2) - initial_leg_height
            i = i + 1
        
        flaag=1
        i=0    
        while(1):
            current = time.time()   #absolute

            if ((current - last_fix) > sample_time_f ) and i < steps:        
                last_fix = current
                trans1,hip1,knee1 = gait.inverse_kinematics_3d_v6(x_fixed[i], y_plane_f[i], y_fixed[i],0 ,legfix_Prev_angs[1], legfix_Prev_angs[2] ,L_f)
                trans3,hip3,knee3 = gait.inverse_kinematics_3d_v6(xnew[i], y_plane[i], ynew[i],0 ,legvar_Prev_angs[1], legvar_Prev_angs[2], L_v )

                if flaag == 1:
                    trans4,hip4,knee4 = gait.generalbasemover_modifed(4, 'f',st ,steps)
                    trans2,hip2,knee2 = gait.generalbasemover_modifed(2, 'f',st ,steps)
                    flaag = 2

                #Publish Fixed leg point               
                set_angle(0,trans1)
                set_angle(1 , hip1)
                set_angle(2, knee1)   
                set_angle(6,trans3)
                set_angle(7 , hip3)
                set_angle(8, knee3) 
                # Move Body                                    
                set_angle(3,trans2[i])
                set_angle(4, hip2[i])
                set_angle(5, knee2[i])         
                set_angle(9,trans4[i])
                set_angle(10 , hip4[i])
                set_angle(11, knee4[i]) 

                i = i + 1
                t = t + sample_time_f
            if (i == steps):
                flag_start =0
                break      

    else:              #even legs pair
        initial_distance_f = leg_pos2_hip[0]
        initial_distance = leg_pos4_hip[0]
        legfix_Prev_angs = leg2_ang
        legvar_Prev_angs = leg4_ang
        y_offset_f=leg_pos2_hip[1]                          #leg2
        y_offset=leg_pos4_hip[1]                             #leg4 
        y_plane_f=np.full((steps+1, 1), y_offset_f)  
        y_plane=np.full((steps+1, 1), y_offset)                      
        L_f='r'
        L_v='l'        

        if  first_step_flag==1  :
                if initial_distance_f >=0:
                    stride_f_mod=stride_f-initial_distance_f
                else: 
                    stride_f_mod=stride_f+np.abs(initial_distance_f)

                if initial_distance >=0:
                    stride_mod=stride_f-initial_distance
                else: 
                    stride_mod=stride_f+np.abs(initial_distance)

                first_step_flag=first_step_flag+1
                

        else:
            error_f=stride_f/2 -np.abs(initial_distance_f)
            error=stride/2 -np.abs(initial_distance)

            if error_f >=0:
                    stride_f_mod=stride_f-error_f
            else: 
                    stride_f_mod=stride_f+np.abs(error_f)   

            if error >=0:
                    stride_mod=stride-error
            else: 
                    stride_mod=stride+np.abs(error)
           

        #leg no.2
        xnew = np.zeros([steps, 1], dtype=float)
        t = 0
        i = 0

        for t in np.arange(0, cycle_time_f, sample_time_f):
            xnew[i] = (stride_f_mod * ((t / cycle_time_f) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time_f)))) - (stride_f_mod / 2) + stride_f_mod / 2) + initial_distance_f
            i = i + 1

        i = 0
        ynew = np.zeros([steps, 1], dtype=float)

        # First part of Ynew in piecewise
        for t in np.arange(0, cycle_time_f / 2, sample_time_f):
            ynew[i] = (-(h_f / (2 * np.pi)) * np.sin(((4 * np.pi) / cycle_time_f) * t) + ((2 * h_f * t) / cycle_time_f) - (h_f / 2)) + (h_f / 2) - initial_leg_height_f
            i = i + 1
        # second part of Ynew in piecewise
        n = (cycle_time_f / 2)
        for t in np.arange(n, cycle_time_f, sample_time_f):
            ynew[i] = (-(h_f / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time_f) * t)) - ((2 * h_f * t) / cycle_time_f) + ((3 * h_f) / 2)) + (h_f / 2) - initial_leg_height_f
            i = i + 1

        x_fixed = xnew
        y_fixed = ynew

        #leg no.4
        i = 0
        t = 0
        xnew = np.zeros([steps, 1], dtype=float)

        for t in np.arange(0, cycle_time, sample_time):
            xnew[i] = (stride_mod * ((t / cycle_time) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time)))) - (stride_mod / 2) + stride_mod / 2) + initial_distance
            i = i + 1

        i = 0
        ynew = np.zeros([steps, 1], dtype=float)

        # First part of Ynew in piecewise
        for t in np.arange(0, cycle_time / 2, sample_time):
            ynew[i] = (-(h / (2 * np.pi)) * np.sin(((4 * np.pi) / cycle_time) * t) + ((2 * h * t) / cycle_time) - (h / 2)) + (h / 2) - initial_leg_height
            i = i + 1
        # second part of Ynew in piecewise
        n = (cycle_time / 2)
        for t in np.arange(n, cycle_time, sample_time):
            ynew[i] = (-(h / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time) * t)) - ((2 * h * t) / cycle_time) + ((3 * h) / 2)) + (h / 2) - initial_leg_height
            i = i + 1
        
        flaag=1
        i=0    
        while(1):
            current = time.time()   #absolute

            if ((current - last_fix) > sample_time_f ) and i < steps:        
                last_fix = current
                trans2,hip2,knee2 = gait.inverse_kinematics_3d_v6(x_fixed[i], y_plane_f[i], y_fixed[i],0 ,legfix_Prev_angs[1], legfix_Prev_angs[2],L_f )
                trans4,hip4,knee4 = gait.inverse_kinematics_3d_v6(xnew[i], y_plane[i], ynew[i],0 ,legvar_Prev_angs[1], legvar_Prev_angs[2],L_v )

                if flaag == 1:
                    trans3,hip3,knee3 = gait.generalbasemover_modifed(3, 'f',st ,steps)
                    trans1,hip1,knee1 = gait.generalbasemover_modifed(1, 'f',st ,steps)
                    flaag = 2

                #Publish Fixed leg point               
                set_angle(3,trans2)
                set_angle(4, hip2)
                set_angle(5, knee2)   
                set_angle(9,trans4)
                set_angle(10, hip4)
                set_angle(11, knee4) 
                # Move Body                                    
                set_angle(0,trans1[i])
                set_angle(1, hip1[i])
                set_angle(2, knee1[i])         
                set_angle(6,trans3[i])
                set_angle(7, hip3[i])
                set_angle(8, knee3[i]) 

                i = i + 1
                t = t + sample_time_f
            if (i == steps):
                flag_start =0
                break   


def trueangle(two_angles,current_angle):
    diff = np.array((2,1),dtype=np.float)
    diff[0] = abs(current_angle - two_angles[0])
    diff[1] = abs(current_angle - two_angles[1])
    if diff[0] < diff[1]:
        return two_angles[0]
    else:
        return two_angles[1]

def set_angle(joint, angle):
    global pub
    msg=str(joint) + ' ' + str(angle)
    sendangle = float(angle)
    msg = str(joint)+ ' ' + str(sendangle)
    pub.publish(msg)

#####################################################################################################################
# Main
if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('setter', String, queue_size=10)
    gait.ros.ros_init(1)
    rospy.Subscriber('fwd', Float32MultiArray, leg_pos)
    rospy.Subscriber('getter', Float32MultiArray , imudata)
    time.sleep(2)
    rate = rospy.Rate(100)  # 10hz 
    
    while not rospy.is_shutdown():
        
        if first_step_flag == 1:
            time.sleep(delay_seq)        
            Gate_Publisher(2 ,75)

            time.sleep(delay_seq)         

            Gate_Publisher(1,150)  
            first_step_flag=0    
        
        else:
            time.sleep(delay_seq)        
            Gate_Publisher(2 ,150)

            time.sleep(delay_seq)         

            Gate_Publisher(1,150)  
            first_step_flag=0    
            

        #rate.sleep()        