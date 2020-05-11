# -*- coding: utf-8 -*-
#!/usr/bin/env python3
from Main_Functions import GP2_Function_V7 as gait
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
draw=False
acc_test=False
x_arr=[]
y_arr=[]
z_arr=[]
stridelen=100
stps=100

if(draw==True):
    for theta1 in np.arange(0,3.14,0.01):
        for theta2 in np.arange(0,3.14,0.01):
            for theta3 in np.arange(0,3.14,0.01):
                x,y,z=gait.forward_kinematics_V3(theta1,theta2,theta3)
                x_arr.append(x)
                y_arr.append(y)
                z_arr.append(z)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_wireframe(x_arr,y_arr,z_arr, rstride=2, cstride=2)

    plt.show()
    print("Done")
                
hip,knee=gait.get_initial_angels(1,0,gait.initalheight)
trans=0
last_x=0
last_y=gait.a
last_z=gait.initalheight
if(acc_test==True):
    transverse_arr,hip_arr,knee_arr,delay=gait.Leg_Ellipse_Trajectory(last_z,stridelen,'f',trans,hip,knee,last_x,last_y)
    last_tr=transverse_arr[len(transverse_arr)-1]
    last_hip=hip_arr[len(hip_arr)-1]
    last_knee=knee_arr[len(knee_arr)-1]
    x,y,z=gait.forward_kinematics_V3(last_tr,last_hip,last_knee)
    print ("X IS "+str(x))
    print ("Y IS "+str(y))
    print ("Z IS "+str(z))
    l_t,l_hip,l_knee=gait.virtual_general_base_mover([x,y,z],last_tr,last_hip,last_knee,stps,stridelen)
    x,y,z=gait.forward_kinematics_V3(l_t,l_hip,l_knee)
    print ("X2 IS "+str(x))
    print ("Y2 IS "+str(y))
    print ("Z2 IS "+str(z))
    last_x=x
    last_y=y
    last_z=-z
    trans=l_t
    hip=l_hip
    knee=l_knee

last_ang1_vel=0
last_ang1_acc=0
vel_arr_rad=[]
acc_arr_rad=[]
last_ang2_vel=0
last_ang2_acc=0
transverse_arr,hip_arr,knee_arr,delay=gait.Leg_Ellipse_Trajectory(last_z,stridelen,'f',trans,hip,knee,last_x,last_y,0.8)
last_ang1=hip_arr[0]
stp_time=delay
for ang in hip_arr:
    curr_ang1_vel=float((ang-last_ang1)/(stp_time))
    vel_arr_rad.append(float(curr_ang1_vel))

    curr_ang1_acc=float((curr_ang1_vel-last_ang1_vel)/stp_time)
    acc_arr_rad.append(float(curr_ang1_acc))

    last_ang1=ang
    last_ang1_vel=curr_ang1_vel


print("VEL ARR IS ")
print(vel_arr_rad)

print("ACC R IS ")
print(acc_arr_rad)