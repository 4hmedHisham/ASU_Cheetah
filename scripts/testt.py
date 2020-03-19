






if __name__ == '__main__':
#!/usr/bin/env python
	import rospy
	import time
	import GP2_Vrep_V3 as v
	import numpy as np
	from std_msgs.msg import Float32MultiArray
	import R2AA as ros


	while(1):


		angles=ros.return_ang()
		print(angles)
		time.sleep(2)



