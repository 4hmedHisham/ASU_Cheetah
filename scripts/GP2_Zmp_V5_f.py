#!/usr/bin/env python

import rospy
import numpy as np
import sympy as sp
import math
from numpy import sin , cos
# import torch
import scipy.linalg
from sympy.solvers import solve
from sympy import Symbol, Eq
from sympy.solvers.solveset import linsolve
from timeit import default_timer as time
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

# Constants
g = 9.81
Mx = 2
My = 3
Mz = 5
Fx = 8
Fy = 12
Fz = 9
hipOffset = 300.78
mass = 20
z = 0
var =0
var2 = 0

#inert = 




# Parameters:
l1 =245
l2 =208.4
a = 112.75
initial_leg_height = 390 # from ground to joint
stride = 150
h = 100     # maximum height of trajectory
cycle_time = 0.3 # total time for each cycle
steps = 20 # number of steps 'even'
initial_distance = 0 # along x

x_fixed = np.zeros([steps, 1], dtype=float)
y_fixed = np.zeros([steps, 1], dtype=float)

x_fixed = np.zeros([steps, 1], dtype=float)
y_fixed = np.zeros([steps, 1], dtype=float)

initial_leg_height_f = 390
stride_f = 150
h_f = 100
cycle_time_f = 0.3
initial_distance_f = 0
sample_time_f = None
pub = 0


#initial position relative to hip and cg
leg1_initial_hip=[]
leg2_initial_hip=[]
leg3_initial_hip=[]
leg4_initial_hip=[]

leg1_initial_cg=[]
leg2_initial_cg=[]
leg3_initial_cg=[]
leg4_initial_cg=[]

#Subscribed data
lin_acc=[]
Ang_acc=[]
lin_acc_prev=[]
Ang_acc_prev=[]
linear_acc_threshold = 20 
angular_acc_threshold = 20
leg_pos3_hip=[] 
leg_pos4_hip=[]
leg_pos1_hip=[]
leg_pos2_hip=[] 

leg_pos3_cg=[] 
leg_pos4_cg=[]
leg_pos1_cg=[]
leg_pos2_cg=[]

legfix_Prev_angs = []
legvar_Prev_angs = []


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

    
    linear_acc_threshold = 10
    angular_acc_threshold = 10
    
    lin_acc_prev=lin_acc    ######save previous imu readings
    Ang_acc_prev=Ang_acc
    
    lin_acc = Array[36:39]  ######get new readings
    Ang_acc = Array[39:42]
    leg3_ang = Array[0:3]
    leg4_ang = Array[3:6]
    leg1_ang = Array[6:9]
    leg2_ang = Array[9:12]    
  
    for i in range (lin_acc):   ###### thresholding  between previous and new readings
        if ((np.absolute(lin_acc[i]-lin_acc_prev[i]))>linear_acc_threshold) or ((np.absolute(Ang_acc[i]-Ang_acc_prev[i]))> angular_acc_threshold):
            z = 1 
        else:
            z = 0


#########################################################################################################

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

def Get_ZMP():
    global mass
    Pc = [0, 0, 0]  # Position of Com relative to origin(cg)
    G = [0, 0, -g]  # Gravity
    #Fi=[Fx,Fy,Fz]                  #Inertia Force from IMU
    Fi = lin_acc * mass
    #Mi = [Mx, My, Mz]              #Inertia Moment
   # Mi = Ang_acc * inert
    Zval = 10  # this will be Z of endeffector from origin which is = Zzmp
    Xzmp, Yzmp, Zzmp = sp.symbols("Xzmp,Yzmp,Zzmp")
    Pzmp = np.array([Xzmp, Yzmp, Zzmp])  # Position of Zmp relative to origin

    A = np.cross(np.subtract(Pc, Pzmp), Fi)
    B = np.cross((np.subtract(Pc, Pzmp)), G)
    F = A + B 
    eq1 = Eq(F[0])
    eq1 = eq1.subs(Zzmp, Zval)
    eq2 = Eq(F[1])
    eq2 = eq2.subs(Zzmp, Zval)
    eq3 = Eq(F[2])
    eq3 = eq3.subs(Zzmp, Zval)
    sol = solve((eq1, eq2, eq3), (Xzmp, Yzmp, Zzmp))

    return sol[Xzmp], sol[Yzmp], Zval


def Mod_contact(leg_no, leg_pos1, leg_pos2_y):

    hipoffset = 300.78
    # leg_pos1 is Position of one leg relative to origin(cg)
    Pzmp = Get_ZMP()
    # Pzmp=[4,5]
    m = (Pzmp[1] - leg_pos1[1]) / (Pzmp[0] - leg_pos1[0])  # Slope of needed line
    # Solve eq y-y1=m(x-x1) with y=cons from the other leg(leg_pos2)
    X_leg = (leg_pos2_y[1] - leg_pos1[1] + m * leg_pos1[0]) / m
    if leg_no==3 or leg_no==4:
    	X_leg = X_leg - hipoffset
    elif leg_no==1 or leg_no==2: #check axisss
    	X_leg = hipoffset- X_leg

    return X_leg 



def Gate_Publisher(leg_no,legfix_initial_hip,legvar_initial_hip,legfix_initial_cg,legvar_initial_cg,legfix_Prev_angs,legvar_Prev_angs):

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

    global x_fixed
    global y_fixed
    global var 
    global var2 


    x_current = 0 
    y_current = 0
    x_current_f =0
    indx_fix = 0
    t_left = 0
    current = 0
    last_fix = 0
    sample_time = cycle_time / steps  # sample time
    sample_time_f = cycle_time_f / steps  # sample time


    if leg_no == 4 or leg_no == 3:    

        xnew = np.zeros([steps, 1], dtype=float)
        t = 0;
        i = 0;
        initial_distance_f = legfix_initial_hip[0]

        for t in np.arange(0, cycle_time_f, sample_time_f):
            xnew[i] = (stride_f * ((t / cycle_time_f) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time_f)))) - (stride_f / 2) + stride_f / 2) + initial_distance_f
            i = i + 1;
        xnew = xnew


  
        i = 0;
        ynew = np.zeros([steps, 1], dtype=float)

        # First part of Ynew in piecewise
        for t in np.arange(0, cycle_time_f / 2, sample_time_f):
            ynew[i] = (-(h_f / (2 * np.pi)) * np.sin(((4 * np.pi) / cycle_time_f) * t) + ((2 * h_f * t) / cycle_time_f) - (h_f / 2)) + (h_f / 2) - initial_leg_height_f
            i = i + 1;

        n = (cycle_time_f / 2)
        for t in np.arange(n, cycle_time_f, sample_time_f):
            ynew[i] = (-(h_f / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time_f) * t)) - ((2 * h_f * t) / cycle_time_f) + ((3 * h_f) / 2)) + (h_f / 2) - initial_leg_height_f
            i = i + 1
        ynew = ynew

        x_fixed = xnew
        y_fixed = ynew

    elif leg_no == 2 or leg_no == 1:
     
        xnew = np.zeros([steps, 1], dtype=float)
        ynew = np.zeros([steps, 1], dtype=float)
        t = 0
        i = 0
        initial_distance=legvar_initial_hip[0]

        if leg_no == 2:
            var = 1
            var2 = 3
        else:
            var = 0
            var2 = 2

        while(1):

            current = time.time()   #absolute

            if ((current - last_fix) > sample_time_f ) and i < steps:        
                last_fix = current
                xnew[i] = (stride * ((t / cycle_time) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time)))) - (stride / 2) + stride / 2) + initial_distance
                
                if t < (cycle_time / 2):

                    ynew[i] = (-(h / (2 * np.pi)) * np.sin(((4 * np.pi) / cycle_time) * t) + ((2 * h * t) / cycle_time) - (h / 2)) + (h / 2) - initial_leg_height

                else:
                    ynew[i] = (-(h / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time) * t)) - ((2 * h * t) / cycle_time) + ((3 * h) / 2)) + (h / 2) - initial_leg_height
         
                #py = 112.75
                #Publish Fixed leg point
                trans,hip,knee = inverse_kinematics_3d_v6(x_fixed[i], 112.75, y_fixed[i],0 ,legfix_Prev_angs[1], legfix_Prev_angs[2] )
                set_angle((var*3),trans)
                set_angle((var*3)+1 , hip)
                set_angle(((var*3)+2), knee)
                #pub = []
                #pub.append(x_fixed[i])
                #pub.append(y_fixed[i])
                #rospy.loginfo(pub)        
                #pub.publish(pub)

                #Publish Variable leg point

                trans,hip,knee = inverse_kinematics_3d_v6(xnew[i], 112.75, ynew[i],0 ,legvar_Prev_angs[1], legvar_Prev_angs[2]) 
                set_angle((var2*3),trans)
                set_angle((var2*3)+1 , hip)
                set_angle((var2*3)+2, knee)
                #rospy.loginfo(xnew[i])            
                #pub.publish(xnew[i])
                #rospy.loginfo(ynew[i])
                #pub.publish(ynew[i])

                if(z == 1):
                    x_current = xnew[i]
                    y_current = ynew[i]
                    #x_current_f = x_fixed[i]
                    t_left = cycle_time_f - t
                    indx_fix = i+1
                    break

                i = i + 1
                t = t + sample_time_f

            if (i == steps):
                break               

        if z == 1:
            #x_moved = legfix_initial_hip[0] - x_current_f
            #t_elabsed = (x_moved/stride_f) * cycle_time_f
            cycletime_required = t_left * 2
            legfix_final_cg = np.zeros([2, 1], dtype=float)
            legvar_final_cg = np.zeros([2, 1], dtype=float)           
            legfix_final_cg[0] = legfix_initial_cg[0] + stride_f     # this is coord of the pt at end of traj for fixed leg
            legfix_final_cg[1] = legfix_initial_cg[1]                # Must be relative to cg
            legvar_final_cg[1] = legvar_initial_cg[1]
            x_target = Mod_contact(leg_no, legfix_final_cg ,legvar_final_cg)    
            trajectory_modification(x_current, y_current, x_target, cycletime_required)
            #stride = stride_mod
            #h = h_mod
            #cycle_time = cycle_time_mod
            #initial_distance = initial_distance_mod       #######################
            #xpub, ypub = trajectory_moved(x_mod, y_mod)
            #initial_distance=0                            #######################




def trajectory_modification(x_current, y_current, x_target, cycle_time):

    global l1
    global l2
    global steps
    global initial_leg_height
    global var
    global var2 

    stride = (x_target - x_current) * 2
    h = y_current  # maximum height of the trajectory
    x_initial = x_target - stride
    initial_distance = x_initial
    sample_time = cycle_time / steps  # sample time, steps should be even

    xnew = np.zeros([steps/2, 1], dtype=float)
    ynew = np.zeros([steps/2, 1], dtype=float)
    t = 0;
    i = 0;
    #i_f= steps/2
    n = (cycle_time / 2)
    current = 0
    last_fix = 0
    last_var = 0


    while(1):

        current = time.time()   #absolute

        if ((current - last_fix) > sample_time_f ) and indx_fix < steps :        #Publish Fixed leg point
            last_fix = current
            trans,hip,knee = inverse_kinematics_3d_v6(x_fixed[indx_fix], 112.75, y_fixed[indx_fix],0 ,legfix_Prev_angs[1], legfix_Prev_angs[2] )
            set_angle((var*3),trans )
            set_angle((var*3)+1 , hip)
            set_angle(((var*3)+2), knee)
            #rospy.loginfo(x_fixed[indx_fix])        
            #pub.publish(x_fixed[indx_fix])          
            #rospy.loginfo(y_fixed[indx_fix])
            #pub.publish(y_fixed[indx_fix])

            indx_fix = indx_fix + 1

        if ((current - last_var) > sample_time ) and i < steps/2 :               #Publish Variable leg point
            last_var = current
            xnew[i] = (stride * ((t / cycle_time) - ((1 / (2 * np.pi)) * np.sin(2 * np.pi * (t / cycle_time)))) - (stride / 2) + stride / 2) + initial_distance
            ynew[i] = (-(h / (2 * np.pi)) * np.sin(4 * np.pi - (((4 * np.pi) / cycle_time) * t)) - ((2 * h * t) / cycle_time) + ((3 * h) / 2)) + (h / 2) - initial_leg_height            
            trans,hip,knee = inverse_kinematics_3d_v6(xnew[i], 112.75, ynew[i],0 ,legvar_Prev_angs[1], legvar_Prev_angs[2])      
            set_angle((var2*3),trans )
            set_angle((var2*3)+1 , hip)
            set_angle(((var2*3)+2), knee)
            #rospy.loginfo(xnew[i])                 
            #pub.publish(xnew[i])          
            #rospy.loginfo(ynew[i])
            #pub.publish(ynew[i])
            i = i + 1
            t = t + sample_time

        if (indx_fix == steps) and i == steps/2:

            break

         
    #return xnew, ynew, stride, h, cycle_time, initial_distance

def inverse_kinematics_3d_v6(px,py,pz,ptran,phip,pknee):
    two_angles = np.array((2, 1))
    u = np.sqrt((- a**2 + py**2 + pz**2))
    x = 2*np.arctan((pz - u)/(a + py))
    y = 2*np.arctan((pz + u)/(a + py))

    if np.abs(x-ptran) < np.abs(y-ptran):
        theta1 = x
    else:
        theta1 = y

    r = px**2 + py**2 + pz**2
    ratio = ((r - a**2 - l1**2 - l2**2) / (2*l1*l2))
    two_angles = acos2(ratio)
    theta3 = trueangle(two_angles, pknee)

    N = l2*cos(theta3) + l1
    num = px*N - l2*sin(theta1)*sin(theta3)*py + l2*sin(theta3)*cos(theta1)*pz
    den = l2*N*cos(theta3) + l1*N + (l2**2 * (sin(theta3))**2)
    ratio = num/den
    two_angles = acos2(ratio)
    theta2 = trueangle(two_angles, phip)
    return theta1,theta2,theta3

def asin2(x):
    angle = np.arcsin(x)
    angles=np.zeros((2,1))
    if x >= 0:
        angles[0]=angle
        angles[1]=(np.pi-angle)
        return angles 
    else:
        angles[0]=angle
        angles[1]=-(np.pi) - angle
        return angles
    
def acos2(x):
    angle = np.arccos(x)
    
    if x >= 0:
        return angle , - angle
    else:
        return angle , - angle

def trueangle(two_angles,current_angle):
    diff = np.array((2,1),dtype=np.float)
    diff[0] = abs(current_angle - two_angles[0])
    diff[1] = abs(current_angle - two_angles[1])
    if diff[0] < diff[1]:
        return two_angles[0]
    else:
        return two_angles[1]
   
def set_angle(joint,angle):
    global pub
	#msg=str(joint)+' '+str(angle)
	#sendangle = float(angle)  #cause sometimes it came in shape of a single list 
	#msg=str(joint)+' '+str(sendangle)
	#pub.publish(msg)




#####################################################################################################################3

# Main
def talker():

    global pub

    rospy.init_node('zmp', anonymous=True)
    #pub = rospy.Publisher('legdata', Float32MultiArray, queue_size=10)
    pub = rospy.Publisher('setter', String, queue_size=10)

    rospy.Subscriber('fwd', Float32MultiArray, leg_pos)
    rospy.Subscriber('getter', Float32MultiArray,imudata)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():


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

        # First we move leg 1 and leg 2 

        legfix_initial_hip = leg_pos4_hip[0]
        legvar_initial_hip = leg_pos2_hip[0]
        legfix_initial_cg = leg_pos4_cg
        legvar_initial_cg = leg_pos2_cg
        leg4_Prev_angs =  leg4_ang
        leg2_Prev_angs =  leg2_ang        



        Gate_Publisher(4 ,legfix_initial_hip,legvar_initial_hip,legfix_initial_cg,legvar_initial_cg,leg4_Prev_angs,leg2_Prev_angs)
        Gate_Publisher(2 ,legfix_initial_hip,legvar_initial_hip,legfix_initial_cg,legvar_initial_cg,leg4_Prev_angs,leg2_Prev_angs)


        # Second we move leg 3 and leg 4 

        legfix_initial_hip = leg_pos3_hip[0]
        legvar_initial_hip = leg_pos1_hip[0]
        legfix_initial_cg = leg_pos3_cg
        legvar_initial_cg = leg_pos1_cg
        leg3_Prev_angs =  leg3_ang
        leg1_Prev_angs =  leg1_ang         

        Gate_Publisher(3 ,legfix_initial_hip,legvar_initial_hip,legfix_initial_cg,legvar_initial_cg,leg3_Prev_angs,leg1_Prev_angs)
        Gate_Publisher(1 ,legfix_initial_hip,legvar_initial_hip,legfix_initial_cg,legvar_initial_cg,leg3_Prev_angs,leg1_Prev_angs)


        #x3 , y3 = linear_traj()
        #x4 , y4 = linear_traj()
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

