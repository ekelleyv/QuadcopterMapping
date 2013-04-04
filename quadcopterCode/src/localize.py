#!/usr/bin/env python
 
import sys
import time
import math
#ROS related initializations
import roslib
import rospy
import os
roslib.load_manifest('ardrone_autonomy')
from std_msgs.msg import String, Float32
from particlefilter import *
from ardrone_autonomy.msg import *
from visualization_msgs.msg import *

#SHOULD BE PUBLISHING A TRANSFORM
class localize:
	def __init__(self):
		self.time = time.time()
		self.pf = particlefilter()

	#NEED TO ADD HEADING INFORMATION
	def update(self, data):
		#Get delta t and update tm
		delta_t = time.time() - self.time #Time in seconds
		self.time = time.time()
		self.pf.propogate(delta_t, data.ax, data.ay, data.az, data.rotZ)
		self.pf.correct(delta_t, data.vx, data.vy, data.altd, data.magX, data.magY, data.magZ)
	
	def get_markers(self):
		return self.pf.est_marker

	def estimate(self):
		return self.pf.est


if __name__ == "__main__":
	#start the class
	print("Running localize")
	localize_1 = localize()
	localize_1.listener()
