import numpy


def fwd_translate(msg):
    pos_leg=[]
    for i in range(4):
        pos_leg[i]=[msg[i*0],msg[i*1],msg[i*2]]


