


#found pause simulation:
#sim.simxPauseCommunication(clientID,True)
#import matplotlib.pyplot as plt
import math 
import numpy as np
import GP2_Vrep_V3 as v
#import rospy
import vrep 
#from std_msgs.msg import Float32MultiArray
import R2A as ros

#import matplotlib.pyplot as plt
import time

import math
#Parameters:
#clientID=0
l1 =244.59
l2 =208.4
initalheight=390
stride=120
plus2pi=False
stability=True
movement=True
stp = 100

#Functions removed ros functions from here (moduled)
# all_angles=0
# rate=0
# pub=0
# def getFromTopic(data):
# 	global all_angles
# 	all_angles=data.data

	




	
def getjointanglesfromvrep():#transverse,hips,knees

    hipangles = np.zeros((4, 1))
    kneeangles = np.zeros((4, 1))
    transverseangles = np.zeros((4, 1))

    for i in range(hipangles.shape[0]):
        transverseangles[i] = ros.get_angles(0 + 3 * i)
        hipangles[i] = ros.get_angles(1 + 3 * i)
        kneeangles[i] = ros.get_angles(2 + 3 * i)

    return transverseangles, hipangles, kneeangles


def GetEndEffectorPos(transverseangles, hipangles, kneeangles):  # Gets End effectors Positions
    leg_pos_x = [254, 254, -258, -258]  # the endeffector position relative to the cg in the x-axis
    leg_pos_y = [116, -116, -116, +116]  # the endeffector position relative to the cg in the y-axis
    # This array will have the pos. of the 4 endeffector
    pos2cg = np.zeros((4, 3))
    pos2joint = np.zeros((4, 3))

    for i in range(4):
        x = l1 * np.cos(hipangles[i]) + l2 * np.cos(hipangles[i] + kneeangles[i])
        z = l1 * np.sin(hipangles[i]) + l2 * np.sin(hipangles[i] + kneeangles[i])
        pos2cg[i, 0] = (leg_pos_x[i] + x)  # forward/backward (x-axis)
        pos2cg[i, 1] = leg_pos_y[i] - z * np.sin(transverseangles[i])  # tilting (y-axis)
        pos2cg[i, 2] = z * np.cos(transverseangles[i])  # Height  (z-axis)
        pos2joint[i, 0] = (x)  # forward/backward (x-axis)
        pos2joint[i, 1] = -z * np.sin(transverseangles[i])  # tilting (y-axis)
        pos2joint[i, 2] = z * np.cos(transverseangles[i])  # Height  (z-axis)

    return pos2cg, pos2joint


def move_leg(leg, direction):
    #First We Get Joint Angles from vrep
    #stabilitymover(leg,angles_handler)
    if direction == 'f':
        sign = 1
    if direction == 'b':
        sign = -1
    error = 0
    transverses , hips, knees = getjointanglesfromvrep()
    #Checked
    #we get End Effector Position
    legspos2cg,legspos2joint=GetEndEffectorPos(transverses,hips,knees)#effector pos with respect to cg got correct angles 
    if movement==True:
        hipp = ros.get_angles(1+3*(leg-1))
        kneee = ros.get_angles(2+3*(leg-1))

        hippp,kneeee,delay=angles(initalheight,sign*stride,hipp,kneee,legspos2joint[(leg-1),0])
        torquehip=np.zeros((4,hippp.shape[0]))
        torqueknee=np.zeros((4,hippp.shape[0]))
        iteration = np.zeros(hippp.shape[0])
        for i in range(hippp.shape[0]):
#            if leg==3:
#                    hippp=hippp-np.pi
            ros.set_angle(1+3*(leg-1),hippp[i])
            ros.set_angle(2+3*(leg-1),kneeee[i])
            

            # for k in range (4):
            #     error,torquehip[k,i]=vrep.simxGetJointForce(clientID,int(angles_handler[1+3*(k-1)]),vrep.simx_opmode_streaming)
            #     error,torqueknee[k,i]=vrep.simxGetJointForce (clientID,int(angles_handler[2+3*(k-1)]),vrep.simx_opmode_streaming)

            time.sleep(delay)
           # iteration[i] = i
            
    return torquehip , torqueknee , iteration             


def inverse_kinematics(xh,zh,phip,pknee):#Gets step angle to move delta x delta y on a specified parth

        r=np.sqrt(xh**2+zh**2)
        two_angles = np.array((2,1))
        ratio1=((r**2-l1**2-l2**2)/(2*l1*l2))
        two_angles=acos2(ratio1)
        theta2=trueangle(two_angles,pknee)
        
        ratio2=((l2*np.sin(theta2))/r)
        two_angles=asin2(ratio2)
        u = math.atan2(zh,xh)
        choose= (u - two_angles)
        theta1=trueangle(choose,phip)
        
        return theta1,theta2
    

def angles(intial_leg_height,s,hipangle,kneeangle,xc):#Gets angles for leg trajectory  xc stands for current x
#tehta1=-133 theta2=+97
#    if init=='y':
#        xc=0
#    else:
#        xc=l1*np.cos(hipangle) + l2*np.cos(hipangle+kneeangle)#TEST
    
    #s = 150 #stride lenth
    #H = 100 #height
    T = 0.001 #total time for each cycle
    h = 100#100 #hsmall
    #intial_leg_height = 300 #from ground to joint
    stp=100#100 #number of steps even number for some reason
    St=T/stp #sample time
    xnew = np.zeros([stp,1], dtype=float)
    t=0;
    i=0;
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
 
    i=0;
    initial_hip = hipangle
    initial_knee = kneeangle
    for t in np.arange(0,T, St):

        theta1[i] , theta2[i] = inverse_kinematics(xnew[i] ,ynew[i] ,initial_hip ,initial_knee )
        initial_hip = theta1[i]
        initial_knee = theta2[i]
        i=i+1
    
    return theta1,theta2,St



#Getting Desired Cg Pos for stability
def getnewcg(pos , SwingLegNo):    # pos is array 4*3
    pos  = np.delete(pos , SwingLegNo-1 ,0  )  # deleting the moving leg co-ordinates
    cg = np.zeros(2)
    cg[0] =  (pos[0,0] + pos[1,0] + pos[2,0])/3
    cg[1] =  (pos[0,1] + pos[1,1] + pos[2,1])/3

    return cg    # cg = [x,y]


def stabilitymover(leg):
    
    transverses , hips, knees = getjointanglesfromvrep()
    legspos2cg,legspos2joint=GetEndEffectorPos(transverses,hips,knees)#effector pos with respect to cg got correct angles 
    newcg=getnewcg(legspos2cg,leg)
    hip=np.zeros((4,1))
    knee=np.zeros((4,1))
    transverse=np.zeros((4,1))
    numofsteps=50
    result=not(newcg[0]<10 and newcg[0]>-10 and newcg[1]<10 and newcg[1]>-10)
    if result and stability==True:
        for i in range(numofsteps):  #moves the base
            
            for ii in range(4):#gets required angles for this step-siza
                #,transverse[ii]
                hip[ii],knee[ii]=inverse_kinematics((legspos2joint[ii,0]-(i+1)*(newcg[0]/numofsteps)),(legspos2joint[ii,1]-(i+1)*(newcg[1]/numofsteps)),-legspos2joint[ii,2],'f')
            hip=fix360(hips,hip)
            for iii in range(4):#moves the stepsize determined
                ros.set_angle((0+3*iii),transverse[iii])
                ros.set_angle((1+3*iii),hip[iii])
                ros.set_angle((2+3*iii),knee[iii])

            time.sleep(0.1)
def generalbasemover(direction): #moves base with same length as stride

    if direction == 'f':
        sign = 1
    if direction == 'b':
        sign = -1
            
    iteration = []
    transverses , hips, knees = getjointanglesfromvrep()
    legspos2cg,legspos2joint=GetEndEffectorPos(transverses,hips,knees)#effector pos with respect to cg got correct angles 
    hip=np.zeros((4,1))
    knee=np.zeros((4,1))
    numofsteps=60
    initial_hip = hips
    initial_knee= knees
#    torquehip=np.zeros(numofsteps)
#    torqueknee=np.zeros(numofsteps)
    for i in range(numofsteps):  #moves the base
        iteration.append(i)
        for ii in range(4):#gets required angles for this step-size    
            hip[ii],knee[ii]=inverse_kinematics((legspos2joint[ii,0]-sign*(i+1)*((stride)/numofsteps)) , legspos2joint[ii,2] , initial_hip[ii] , initial_knee[ii])

        initial_hip = hip
        initial_knee = knee

        for iii in range(4):#moves the stepsize determined

            ros.set_angle((1+3*iii),hip[iii])
            ros.set_angle((2+3*iii),knee[iii])
#            torquehip[i]=vrep.simxGetJointForce (clientID,int(angles_handler[1+3*iii)]),hip[iii])
#            torqueknee[i]=vrep.simxGetJointForce (clientID,int(angles_handler[2+3*iii)]),knee[iii])

        time.sleep(0.005)  
        
def trot(seq,modified):
    transverses , hips, knees = getjointanglesfromvrep()
#Checked
#we get End Effector Position
    legspos2cg,legspos2joint=GetEndEffectorPos(transverses,hips,knees)#effector pos with respect to cg got correct angles 

    error,hipp1 = ros.get_angles(1+3*(seq-1))
    error,kneee1 = ros.get_angles(2+3*(seq-1))
    error,hipp2=ros.get_angles(1+3*(seq-1+2))
    error,kneee2=ros.get_angles(2+3*(seq-1+2))
    hippp1,kneeee1,delay1=angles(initalheight,stride,hipp1,kneee1,legspos2joint[(seq-1),0])
    hippp2,kneeee2,delay2=angles(initalheight,stride,hipp2,kneee2,legspos2joint[(seq-1+2),0])

    for i in range(hippp1.shape[0]):
#            if leg==3:
#                    hippp=hippp-np.pi
        ros.set_angle((1+3*(seq-1)),hippp1[i])    
        ros.set_angle((2+3*(seq-1)),kneeee1[i])
        ros.set_angle((1+3*(seq-1+2)),hippp2[i])    
        ros.set_angle((2+3*(seq-1+2)),kneeee2[i])

        if modified=='y':#first legs will move normally, other legs will go forward
            hip=np.zeros((4,1))
            knee=np.zeros((4,1))
            transverse=np.zeros((4,1))
            numofsteps=hippp1.shape[0]            
            if seq==1:
                for ii in range(4):
                    #,transverse[ii]
                    hip[ii],knee[ii]=inverse_kinematics((legspos2joint[ii,0]-(i+1)*((stride)/numofsteps)), legspos2joint[ii,2] , hips[1+3*(seq)] , knees[2+3*seq])
                ros.set_angle((1+3*1),hip[1])
                ros.set_angle((2+3*1),knee[1])
                ros.set_angle((1+3*1),hip[2])
                ros.set_angle((2+3*1),knee[2])
            if seq==2:
                for ii in range(4):
                    #,transverse[ii]
                    hip[ii],knee[ii]=inverse_kinematics((legspos2joint[ii,0]-(i+1)*((stride)/numofsteps)),legspos2joint[ii,2] ,hips[1+3*(seq)] ,knees[2+3*seq] )
                ros.set_angle((1+3*1),hip[0])
                ros.set_angle((2+3*1),knee[0])
                ros.set_angle((1+3*1),hip[3])
                ros.set_angle((2+3*1),knee[3])

        time.sleep(delay2)

def trot2(seq):
    transverses, hips, knees = getjointanglesfromvrep()
    # Checked
    # we get End Effector Position
    legspos2cg, legspos2joint = GetEndEffectorPos(transverses, hips,
                                                  knees)  # effector pos with respect to cg got correct angles
    hipp1 = ros.get_angles(1 + 3 * (seq - 1))
    kneee1 = ros.get_angles(2 + 3 * (seq - 1))
    hipp2 = ros.get_angles(1 + 3 * (seq - 1 + 2))
    kneee2 = ros.get_angles(2 + 3 * (seq - 1 + 2))

    hippp1, kneeee1, delay1 = angles(initalheight, stride, hipp1, kneee1, legspos2joint[(seq - 1), 0])
    hippp2, kneeee2, delay2 = angles(initalheight, stride, hipp2, kneee2, legspos2joint[(seq - 1 + 2), 0])

    if (seq == 1):
        leg_for_base = [1, 3]
        hip3, knee3 = generalbasemover_modifed(leg_for_base[0] + 1)
        hip4, knee4 = generalbasemover_modifed(leg_for_base[1] + 1)
    else:
        leg_for_base = [0, 2]
        hip3, knee3 = generalbasemover_modifed(leg_for_base[0] + 1)
        hip4, knee4 = generalbasemover_modifed(leg_for_base[1] + 1)
    numofsteps = stp

    for i in range(numofsteps):
        ros.set_angle((1 + 3 * (seq - 1)), hippp1[i])
        ros.set_angle((2 + 3 * (seq - 1)), kneeee1[i])
        ros.set_angle((1 + 3 * (seq - 1 + 2)), hippp2[i])
        ros.set_angle((2 + 3 * (seq - 1 + 2)), kneeee2[i])

        ros.set_angle((1 + 3 * (leg_for_base[0])), hip3[i])
        ros.set_angle((2 + 3 * (leg_for_base[0])), knee3[i])
        ros.set_angle((1 + 3 * (leg_for_base[1])), hip4[i])
        ros.set_angle((2 + 3 * (leg_for_base[1])), knee4[i])
        time.sleep(delay2)


def generalbasemover_modifed(leg):  # moves base with same length as stride

    transverses, hips, knees = getjointanglesfromvrep()
    legspos2cg, legspos2joint = GetEndEffectorPos(transverses, hips,
                                                  knees)  # effector pos with respect to cg got correct angles 
    # hip=np.zeros((4,1))
    # knee=np.zeros((4,1))
    transverse = np.zeros((4, 1))
    numofsteps = stp
    hip_collector = np.zeros(numofsteps)
    knee_collector = np.zeros(numofsteps)

    for i in range(numofsteps):  # moves the base    
        hip, knee = inverse_kinematics((legspos2joint[leg - 1, 0] - (i + 1) * (stride / numofsteps)), legspos2joint[leg - 1, 2] , hips[leg-1] , knees[leg-1])
        hip = hip
        hip_collector[i] = hip
        knee_collector[i] = knee
    return hip_collector, knee_collector
def fix360(sensorangle,measuredangles):
    if(sensorangle[0]<0 and measuredangles[0]>0):
        measuredangles=measuredangles-2*np.pi
    if(sensorangle[0]>0 and measuredangles[0]<0):
        measuredangles=measuredangles+2*np.pi
    return measuredangles

def onestepcreeping():
    time.sleep(0.4)
    torquehip, torqueknee, iteration = move_leg(1, 'f')
    # gait.plot_torque(torquehip, torqueknee, iteration)
    time.sleep(0.4)
    torquehip, torqueknee, iteration = move_leg(2, 'f')
    #  gait.plot_torque(torquehip, torqueknee, iteration)
    time.sleep(0.4)
    generalbasemover('f')
    time.sleep(0.4)
    torquehip, torqueknee, iteration = move_leg(3, 'f')
    #   gait.plot_torque(torquehip, torqueknee, iteration)
    time.sleep(0.4)
    torquehip, torqueknee, iteration = move_leg(4, 'f')


hipinit=-2.01842
kneeinit=0.980479
trinit=0       
   
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
    
def plot_torque(hip , knee , iteration):
    
    plt.figure()    
    plt.plot(iteration , hip[0,:],'r')
    plt.plot(iteration , hip[1,:],'b')
    plt.plot(iteration , hip[2,:],'r--')
    plt.plot(iteration , hip[3,:],'b--')

    plt.legend(['leg1 hip' , 'leg2 hip ', 'leg3 hip ', 'leg4 hip '])
    plt.xlabel('trajectory(step)')
    plt.ylabel('Torque(N.m)')
    plt.show()
    
    plt.figure()    
    plt.plot(iteration , knee[0,:],'r')
    plt.plot(iteration , knee[1,:],'b')
    plt.plot(iteration , knee[2,:],'r--')
    plt.plot(iteration , knee[3,:],'b--')

    plt.legend(['leg1 knee' , 'leg2 knee ', 'leg3 knee ', 'leg4 knee '])
    plt.xlabel('trajectory(step)')
    plt.ylabel('Torque(N.m)')
    plt.show()
# def ros_get_angles():
# 	return angles
#ros_init()
#ros_set_ang(2,2.31)
# #print(ros_get_angles())
# ros.ros_init()
# print('WHAT')
# time.sleep(2)
# print("ARE")
# ros.set_angle('bc1',3.1125)
# print("U")
