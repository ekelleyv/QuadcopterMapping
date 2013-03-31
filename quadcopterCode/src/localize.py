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

#SHOULD BE PUBLISHING A TRANSFORM
class localize:
	def __init__(self):
		filename= "/home/ekelley/Dropbox/thesis_data/" + time.strftime('%Y_%m_%d_%H_%M_%S') + ".txt" #in the format YYYYMMDDHHMMSS
		self.f = open(filename, 'w+')
		self.f.write("time, x, y, z, alt\n")
		self.time = time.time()
		self.pf = particlefilter(num_particles=1, linear_noise=0, angular_noise=0)

	#NEED TO ADD HEADING INFORMATION
	def update(self, data):
		#Get delta t and update tm
		delta_t = time.time() - self.time #Time in seconds
		self.time = time.time()
		self.pf.propogate(delta_t, data.vx, data.vy, data.altd, data.rotZ)
		pos_est = self.pf.estimate()
		self.f.write("%d, %f, %f, %f, %f\n" % (self.time, pos_est.x, pos_est.y, pos_est.z, pos_est.theta))

	def estimate(self):
		return self.pf.estimate()


if __name__ == "__main__":
	#start the class
	print("Running localize")
	localize_1 = localize()
	localize_1.listener()
