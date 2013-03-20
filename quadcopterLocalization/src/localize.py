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
from particles import *
from ardrone_autonomy.msg import *

#SHOULD BE PUBLISHING A TRANSFORM
class localize:
	def __init__(self):
		rospy.init_node("localize")
		filename= "/home/ekelley/Dropbox/thesis_data/" + time.strftime('%Y_%m_%d_%H_%M_%S') + ".txt" #in the format YYYYMMDDHHMMSS
		self.f = open(filename, 'w+')
		self.f.write("time, x, y, z, alt\n")
		self.time = time.time()
		self.particles = particles(num_particles=1, linear_noise=0, angular_noise=0)
		self.x_est = rospy.Publisher('localize/x_est', Float32)
		self.y_est = rospy.Publisher('localize/y_est', Float32)
		self.z_est = rospy.Publisher('localize/z_est', Float32)
		self.theta_est = rospy.Publisher('localize/theta_est', Float32)
		self.var = rospy.Publisher('localize/var', Float32)

	#NEED TO ADD HEADING INFORMATION
	def callback(self, data):
		#Get delta t and update tm
		delta_t = time.time() - self.time #Time in seconds
		self.time = time.time()
		self.particles.propogate(delta_t, data.vx, data.vy, data.vz, 0)
		pos_est = self.particles.estimate()
		self.x_est.publish(pos_est.x)
		self.y_est.publish(pos_est.y)
		self.z_est.publish(pos_est.z)
		self.f.write("%d, %f, %f, %f, %f\n" % (self.time, pos_est.x, pos_est.y, pos_est.z, data.altd))
		self.var.publish(0)


    
	def listener(self):
		rospy.Subscriber("/ardrone/navdata", Navdata, self.callback)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()


if __name__ == "__main__":
	#start the class
	print("Running localize")
	localize_1 = localize()
	localize_1.listener()
