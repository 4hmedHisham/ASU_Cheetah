import GP2_Function_V7 as gait 
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

x_arr=[]
y_arr=[]
z_arr=[]

for theta1 in np.arange(0,3.14,0.001):
    for theta2 in np.arange(0,3.14,0.001):
        for theta3 in np.arange(0,3.14,0.001):
            x,y,z=gait.forward_kinematics_V3(theta1,theta2,theta3)
            x_arr.append(x)
            y_arr.append(y)
            z_arr.append(z)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_wireframe(x,y,z, rstride=2, cstride=2)

plt.show()
print("Done")
            
