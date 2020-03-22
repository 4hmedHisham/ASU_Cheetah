#!/usr/bin/env python3

import R2A as ros
import time







ros.ros_init()
while(1):

	time.sleep(2)
	ros.print_ang()
