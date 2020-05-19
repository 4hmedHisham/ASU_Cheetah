#!/usr/bin/env python
import numpy as np
import sympy as sp
import math
from numpy import sin , cos
import scipy.linalg
from sympy.solvers import solve
from sympy import Symbol, Eq
from timeit import default_timer as time
import time
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

x=[1,2,4,5]
y=len(x)
z=x[y-1]
print("last element "+str(z))