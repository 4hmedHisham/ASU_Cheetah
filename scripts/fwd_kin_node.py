import GP2_Function_V3 as Alg
import NodeMaker as make
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import rospy
import time
from itertools import chain 
#Trying SOmething Pls Work
intial_name = ['ab3', 'bc3', 'cd3', 'ab4', 'bc4', 'cd4', 'ab1', 'bc1', 'cd1', 'ab2', 'bc2', 'cd2']
legspos2cg=[]
legspos2joint=[]
alldata=[]
flag=True
Fwd_node=None
flatten_list=None
onetime=True

def fwd_kin(data):

    
    global flag
    global legspos2cg
    global legspos2joint
    global all_data
    global Fwd_node
    global flatten_list
    global onetime
    # if(flag):w
    #     time.sleep(1)
    #     flag=False
    #     print(Fwd_node)
    all_angels=data.data[:12]
    hipangles = np.zeros((4, 1))
    kneeangles = np.zeros((4, 1))
    transverseangles = np.zeros((4, 1))
    for i in range(hipangles.shape[0]):
        transverseangles[i] = all_angels[0 + 3 * i]
        hipangles[i] = all_angels[1 + 3 * i]
        kneeangles[i] = all_angels[2 + 3 * i]
    pos2cg,pos2joint=Alg.GetEndEffectorPos(transverseangles, hipangles, kneeangles)
    #print('pos2joint is'+str(pos2joint))
    #legspos2cg.append(pos2cg)
    legspos2joint=list(chain.from_iterable(pos2joint))
    legspos2cg=list(chain.from_iterable(pos2cg))
    all_data=legspos2joint+legspos2cg
    #print(all_data)
    #print('pos2joint flat is'+str(legspos2joint))
    # if onetime:
    #     print('begin')
    #     flatten_list = list(chain.from_iterable(legspos2joint))
    #     print(flatten_list)
    #     print('end')
       
fwd_kin=make.ros('b','fwd_kin',['fwd','getter'],fwd_kin)
while not rospy.is_shutdown():
    fwd_kin.ros_publish(all_data)
    
    