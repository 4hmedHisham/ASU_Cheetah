#!/usr/bin/env python

import rospy
import numpy as np
import sympy as sp
import math
from numpy import sin , cos
import scipy.linalg
from sympy.solvers import solve
#from sympy.solvers.solveset import linsolve
from sympy import Symbol,Eq
from timeit import default_timer as time
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import GP2_Function_V7 as gait

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
linear_acc_threshold =4
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

leg_pos3__cg=np.zeros([3, 1], dtype=float) 
leg_pos4__cg=np.zeros([3, 1], dtype=float)
leg_pos1__cg=np.zeros([3, 1], dtype=float)
leg_pos2__cg=np.zeros([3, 1], dtype=float)

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

    lin_acc_prev=lin_acc    ######save previous imu readings
    Ang_acc_prev=Ang_acc
    
    lin_acc = Array[36:39]  ######get new readings
    Ang_acc = Array[39:42]

    leg3_ang = Array[0:3] 
    leg4_ang = Array[3:6]
    leg1_ang = Array[6:9]
    leg2_ang = Array[9:12]    
    # WE CHANGED IT TO 2 TO neglect the z
    for i in range (2):   ###### thresholding  between previous and new readings
        lin_acc_diff=np.absolute(lin_acc[i]-lin_acc_prev[i])
        ang_acc_diff=np.absolute(Ang_acc[i]-Ang_acc_prev[i])
        if (lin_acc_diff>linear_acc_threshold) or (ang_acc_diff> angular_acc_threshold):
            z = 1 
            print("z value changed")
            break
        # else:
        #     z = 0


########################################################################################################

def leg_pos(data):
    Array2 = np.array(data.data)

    global leg_pos3_hip
    global leg_pos4_hip
    global leg_pos1_hip
    global leg_pos2_hip
    global leg_pos3__cg
    global leg_pos4__cg
    global leg_pos1__cg
    global leg_pos2__cg

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

    global leg_pos1__cg
    global leg_pos2__cg
    global leg_pos3__cg
    global leg_pos4__cg

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
    indx_fix=0

    t_left = 0
    current = 0
    last_fix = 0
    sample_time = np.float(cycle_time) / steps # sample time
    sample_time_f =np.float(cycle_time_f) / steps   # sample time
    legfix_final_cg = np.zeros([2, 1], dtype=float)
    legvar_final_cg = np.zeros([2, 1], dtype=float)

  
    stride= st
    stride_f=st
    if pair_no==1 :    #odd number pair 
        
        initial_distance_f = leg_pos3_hip[0]
        initial_distance = leg_pos1_hip[0]
        legfix_Prev_angs = leg3_ang
        legvar_Prev_angs = leg1_ang
        legfix_initial_cg = leg_pos3__cg
        legvar_initial_cg=  leg_pos1__cg
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
           

        #leg no.3
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

        #leg no.1
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
                trans3,hip3,knee3 = gait.inverse_kinematics_3d_v6(x_fixed[i], 112.75, y_fixed[i],0 ,legfix_Prev_angs[1], legfix_Prev_angs[2] )
                trans1,hip1,knee1 = gait.inverse_kinematics_3d_v6(xnew[i], 112.75, ynew[i],0 ,legvar_Prev_angs[1], legvar_Prev_angs[2] )

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

                if(z == 1):
                    x_current = xnew[i]
                    y_current = ynew[i]
                    t_left = cycle_time_f - t
                    indx_fix = i+1
                    break                
                 
                i = i + 1
                t = t + sample_time_f
            if (i == steps):
                flag_start =0
                break

        if z == 1:
            cycletime_required = t_left * 2          
            legfix_final_cg[0] = legfix_initial_cg[0] + stride_f_mod     # this is coord of the pt at end of traj for fixed leg
            legvar_final_cg[0] = legvar_initial_cg[0] + stride_mod
            legfix_final_cg[1] = legfix_initial_cg[1]                    # Must be relative to cg
            legvar_final_cg[1] = legvar_initial_cg[1]
            x_target_fix,x_target_var  = Mod_contact2(1, legfix_final_cg ,legvar_final_cg)  
            #print("x target is "+ str(x_target))  
            trajectory_modification2(x_current, y_current, x_target_fix,x_target_var,cycletime_required,indx_fix,pair_no)
            z=0
              


    else:              #even legs pair
        initial_distance_f = leg_pos4_hip[0]
        initial_distance = leg_pos2_hip[0]
        legfix_Prev_angs = leg4_ang
        legvar_Prev_angs = leg2_ang
        legfix_initial_cg = leg_pos4__cg        
        legvar_initial_cg=  leg_pos2__cg
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
           

        #leg no.4
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
                trans4,hip4,knee4 = gait.inverse_kinematics_3d_v6(x_fixed[i], 112.75, y_fixed[i],0 ,legfix_Prev_angs[1], legfix_Prev_angs[2] )
                trans2,hip2,knee2 = gait.inverse_kinematics_3d_v6(xnew[i], 112.75, ynew[i],0 ,legvar_Prev_angs[1], legvar_Prev_angs[2] )

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
                if(z == 1):
                    x_current = xnew[i]
                    y_current = ynew[i]
                    t_left = cycle_time_f - t
                    indx_fix = i+1
                    break                  
                i = i + 1
                t = t + sample_time_f
            if (i == steps):
                flag_start =0
                break
        if z == 1:
            cycletime_required = t_left * 2
            # legfix_final_cg = np.zeros([2, 1], dtype=float)
            # legvar_final_cg = np.zeros([2, 1], dtype=float)           
            legfix_final_cg[0] = legfix_initial_cg[0] + stride_f_mod     # this is coord of the pt at end of traj for fixed leg
            legvar_final_cg[0] = legvar_initial_cg[0] + stride_mod
            legfix_final_cg[1] = legfix_initial_cg[1]                # Must be relative to cg
            legvar_final_cg[1] = legvar_initial_cg[1]
            x_target_fix,x_target_var  = Mod_contact2(2, legfix_final_cg ,legvar_final_cg)  
            #print("x target is "+ str(x_target))  
            trajectory_modification2(x_current, y_current,x_target_fix,x_target_var,cycletime_required,indx_fix,pair_no)
            z = 0                    

def trajectory_modification2(x_current, y_current, x_target_fix,x_target_var, cycle_time,indx_fix,pair_no):

    global l1
    global l2
    global steps
    global initial_leg_height
    global h
    global sample_time_f        
    global x_fixed
    global y_fixed
    global leg1_ang
    global leg2_ang
    global leg3_ang
    global leg4_ang        

    strideV = (x_target_var - x_current) * 2
    h = y_current  # maximum height of the trajectory
    initial_distanceV = x_target_var - strideV
    sample_time = cycle_time / steps  # sample time, steps should be even

    strideF = (x_target_fix - x_current) * 2
    h = y_current  # maximum height of the trajectory
    initial_distanceF = x_target_fix - strideF
    sample_time = cycle_time / steps  # sample time, steps should be even

    xnew_V = np.zeros([steps/2, 1], dtype=float)
    ynew_V = np.zeros([steps/2, 1], dtype=float)
    xnew_F = np.zeros([steps/2, 1], dtype=float)
    ynew_F = np.zeros([steps/2, 1], dtype=float)   
    t = (cycle_time / 2)
    i = 0
    ii = 0
    current = 0
    last_fix = 0
    last_var = 0
    
    if pair_no == 1 :    #odd number pair         
        varF = 0
        varV = 2
        legfix_Prev_angs = leg3_ang
        legvar_Prev_angs = leg1_ang
    else:
        varF = 1
        varV = 3
        legfix_Prev_angs = leg4_ang
        legvar_Prev_angs = leg2_ang            


    while(1):

        current = time.time()   #absolute        
        if ((current - last_fix) > sample_time ) and ii < steps/2 :        #Publish Fixed leg point
            last_fix = current
            xnew_F[ii] = (strideF * ((t / cycle_time) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time)))) - (strideF / 2) + strideF / 2) + initial_distanceF
            ynew_F[ii] = (-(h / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time) * t)) - ((2 * h * t) / cycle_time) + ((3 * h) / 2)) + (h / 2) - initial_leg_height                        
            trans,hip,knee = gait.inverse_kinematics_3d_v6(xnew_F[ii], 112.75, ynew_F[ii],0 ,legfix_Prev_angs[1], legfix_Prev_angs[2] )
            set_angle((varF*3),trans )
            set_angle((varF*3)+1 , hip)
            set_angle(((varF*3)+2), knee)
            ii = ii + 1
            t = t + sample_time

        if ((current - last_var) > sample_time ) and i < steps/2 :               #Publish Variable leg point
            last_var = current
            xnew_V[i] = (strideV * ((t / cycle_time) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time)))) - (strideV / 2) + strideV / 2) + initial_distanceV
            ynew_V[i] = (-(h / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time) * t)) - ((2 * h * t) / cycle_time) + ((3 * h) / 2)) + (h / 2) - initial_leg_height            
            trans,hip,knee = gait.inverse_kinematics_3d_v6(xnew_V[i], 112.75, ynew_V[i],0 ,legvar_Prev_angs[1], legvar_Prev_angs[2])    
            set_angle((varV*3),trans )
            set_angle((varV*3)+1 , hip)
            set_angle(((varV*3)+2), knee)
            i = i + 1
            t = t + sample_time

        if (ii == steps/2) and (i == steps/2):
            print("Modification Done !!")
            break

def trajectory_modification(x_current, y_current, x_target, cycle_time,indx_fix,pair_no):

    global l1
    global l2
    global steps
    global initial_leg_height
    global h
    global sample_time_f        
    global x_fixed
    global y_fixed
    global leg1_ang
    global leg2_ang
    global leg3_ang
    global leg4_ang        

    stride = (x_target - x_current) * 2
    h = y_current  # maximum height of the trajectory
    initial_distance = x_target - stride
    sample_time = cycle_time / steps  # sample time, steps should be even

    xnew = np.zeros([steps/2, 1], dtype=float)
    ynew = np.zeros([steps/2, 1], dtype=float)
    t = (cycle_time / 2)
    i = 0
    current = 0
    last_fix = 0
    last_var = 0
    
    if pair_no == 1 :    #odd number pair         
        varF = 0
        varV = 2
        legfix_Prev_angs = leg3_ang
        legvar_Prev_angs = leg1_ang
    else:
        varF = 1
        varV = 3
        legfix_Prev_angs = leg4_ang
        legvar_Prev_angs = leg2_ang            


    while(1):

        current = time.time()   #absolute        
        if ((current - last_fix) > sample_time_f ) and indx_fix < steps :        #Publish Fixed leg point
            last_fix = current
            trans,hip,knee = gait.inverse_kinematics_3d_v6(x_fixed[indx_fix], 112.75, y_fixed[indx_fix],0 ,legfix_Prev_angs[1], legfix_Prev_angs[2] )
            set_angle((varF*3),trans )
            set_angle((varF*3)+1 , hip)
            set_angle(((varF*3)+2), knee)
            indx_fix = indx_fix + 1

        if ((current - last_var) > sample_time ) and i < steps/2 :               #Publish Variable leg point
            last_var = current
            xnew[i] = (stride * ((t / cycle_time) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time)))) - (stride / 2) + stride / 2) + initial_distance
            ynew[i] = (-(h / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time) * t)) - ((2 * h * t) / cycle_time) + ((3 * h) / 2)) + (h / 2) - initial_leg_height            
            trans,hip,knee = gait.inverse_kinematics_3d_v6(xnew[i], 112.75, ynew[i],0 ,legvar_Prev_angs[1], legvar_Prev_angs[2])    
            set_angle((varV*3),trans )
            set_angle((varV*3)+1 , hip)
            set_angle(((varV*3)+2), knee)
            i = i + 1
            t = t + sample_time

        if (indx_fix == steps) and i == steps/2:
            print("Modification Done !!")
            break
          
def Get_ZMP():

    global mass
    global g
    global lin_acc
    global Ang_acc
    Pc=np.array([0,0,0])
    G=np.array([0,0,mass*-g])
    #Pc = [0, 0, 0]               # Position of Com relative to origin(cg)
    # G = [0, 0, -g]               # Gravity
    # G = G * mass     
    lin_acc1 = np.asarray(lin_acc)
    lin_acc1 = np.rint(lin_acc1)            
    Fi = lin_acc1 * mass          #Inertia Force from IMU           
    #Mi = Ang_acc * inert         #Inertia Moment
    Zval = -390                  # this will be Z of endeffector from origin which is = Zzmp
    Xzmp, Yzmp, Zzmp = sp.symbols("Xzmp,Yzmp,Zzmp")
    Pzmp = np.array([Xzmp, Yzmp, Zzmp])  # Position of Zmp relative to origin
    A = np.cross(np.subtract(Pc, Pzmp), Fi)
    B = np.cross((np.subtract(Pc, Pzmp)), G)
    F = A + B 
    eq1 = Eq(F[0],0)
    eq1 = eq1.subs(Zzmp, Zval)
    eq2 = Eq(F[1],0)
    eq2 = eq2.subs(Zzmp, Zval)
    eq3 = Eq(F[2],0)
    eq3 = eq3.subs(Zzmp, Zval)
    sol = solve((eq1, eq2, eq3), (Xzmp, Yzmp, Zzmp))
    #print(sol[Xzmp])
    return sol[Xzmp], sol[Yzmp], Zval


def Mod_contact(leg_no, leg_pos1, leg_pos2_y):

    hipoffset = 253
    # leg_pos1 is Position of one leg relative to origin(cg)
    Pzmp = Get_ZMP()
    # Pzmp=[4,5]
    m = (Pzmp[1] - leg_pos1[1]) / (Pzmp[0] - leg_pos1[0])  # Slope of needed line
    # Solve eq y-y1=m(x-x1) with y=cons from the other leg(leg_pos2)
    X_leg = (leg_pos2_y[1] - leg_pos1[1] + m * leg_pos1[0]) / m
    # if leg_no==3 or leg_no==4:
    X_leg = X_leg - hipoffset
    # elif leg_no==1 or leg_no==2: #check axisss
        # X_leg = hipoffset- X_leg
    print(X_leg)
    return X_leg 


def Mod_contact2(leg_no, legfix_final_cg ,legvar_final_cg):

    global leg_pos3__cg
    global leg_pos4__cg
    global leg_pos1__cg
    global leg_pos2__cg
    flag_good_pos =0
    hipoffset = 253
    if leg_no == 2:
        legvar_Curr_cg = leg_pos2__cg
    else:
        legvar_Curr_cg = leg_pos1__cg    
    #leg_pos1 is Position of one leg relative to origin(cg)
    y1 = legvar_final_cg[1]
    y2 = legfix_final_cg[1] 
    Pzmp=Get_ZMP()
    x1_cgF = legvar_final_cg[0]
    x1_cgC = legvar_Curr_cg[0]
    x = np.abs(x1_cgF - x1_cgC)
    while(flag_good_pos==0):
        x1 = (x) + x1_cgC
        #print(x1)
        m=((Pzmp[1]-y1) / (Pzmp[0]-x1))       #Slope of needed line
        #print(m)
        c = y1-m*x1
        #Solve eq y=mx+c with y=cons from the other leg(leg_pos2)
        x2 = (y2-c) / m
        x_target_fix = x2 - hipoffset
        x_target_var = x1 + hipoffset
        if x_target_fix <150 and x_target_fix > -150 and x_target_var<150 and x_target_var > -150:
            flag_good_pos=1
        else:
            x= x-1

    print(x_target_fix)
    print(x_target_var)
    return x_target_fix, x_target_var


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
