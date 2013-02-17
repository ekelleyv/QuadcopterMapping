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

class localize:
	def __init__(self):
		rospy.init_node("localize")
		self.time = time.time()
		self.particles = particles(num_particles=1, linear_noise=0, angular_noise=0)
		self.x_est = rospy.Publisher('localize/x_est', Float32)
		self.y_est = rospy.Publisher('localize/y_est', Float32)
		self.z_est = rospy.Publisher('localize/z_est', Float32)
		self.var = rospy.Publisher('localize/var', Float32)

	def callback(self, data):
		#Get delta t and update tm
		delta_t = time.time() - self.time #Time in seconds
		self.time = time.time()

		# Getting heading from 3d magnetometer
		# stackoverflow.com/questions/1055014
		#NOT FUNCTIONAL YET
		# heading = 0;
		# if (data.magY > 0):
		# 	heading = 90 - math.atan2(data.magX, data.magY)/math.pi
		# elif (data.magY < 0):
		# 	heading = 270 - math.atan2(data.magX, data.magY)/math.pi
		# elif ((data.magY==0) and (data.magX < 0)):
		# 	heading = 180
		# elif ((data.magY==0) and (data.magX > 0)):
		# 	heading = 0

		# print "Mag %d %d %d" % (data.magX, data.magY, data.magZ)
		# print "Heading: %f" % heading
		# print "(%f, %f, %f, %f)" % (delta_t, data.vx, data.vy, data.vz)
		self.particles.propogate(delta_t, data.vx, data.vy, data.vz, 0)
		pos_est = self.particles.estimate()
		self.x_est.publish(pos_est.x)
		self.y_est.publish(pos_est.y)
		self.z_est.publish(pos_est.z)
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
