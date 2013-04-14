#!/usr/bin/env python

import sys
import time
import math
import numpy
#ROS related initializations
import roslib; roslib.load_manifest('quadcopterCode')
import rospy
import tf
import os
from std_msgs.msg import String, Float32
from particlefilter import *
from visualization_msgs.msg import *
from ardrone_autonomy.msg import *

#SHOULD BE PUBLISHING A TRANSFORM
class localize:
	def __init__(self):
		self.last_timestamp = 0
		self.last_time = time.time()
		self.delay = 0
		# Noisy no resample
		# self.pf = particlefilter(num_particles=100, vis_noise = 100, ultra_noise = 100, linear_noise = 0, angular_noise = .01, ar_noise=100, ar_resample_rate=0, ar_resample=False)
		# No noise resample
		# self.pf = particlefilter(num_particles=10, vis_noise = 0, ultra_noise = 0, linear_noise = 0, angular_noise = 0, ar_noise=100, ar_resample_rate=0, ar_resample=True)
		
		#Noisy resample
		self.pf = particlefilter(num_particles=100, vis_noise = 100, ultra_noise = 100, linear_noise = 0, angular_noise = .01, ar_noise=100, ar_resample_rate=0, ar_resample=True)
		self.first_update = True


	# Recieve data from ROS msgs
	def listener(self):
		rospy.Subscriber("/ardrone/navdata", Navdata, self.update)
		rospy.Subscriber("/visualization_marker", Marker, self.ar_correct)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def update(self, data):
		#Get delta t and update tm
		timestamp = data.tm * .000001 #Convert microseconds to seconds
		new_time = time.time()
		if (self.first_update):
			self.last_time = new_time
			self.last_timestamp = timestamp
			self.first_update = False
			return

		delta_timestamp = timestamp - self.last_timestamp #Time in seconds
		delta_time = new_time - self.last_time
		self.delay += delta_time - delta_timestamp
		# print "Percent Realtime= %3.0f%% | Total delay: %2.2f | Delta timestamp: %1.6f | Delta time: %1.6f" % ((delta_timestamp/delta_time) * 100, abs(self.delay), delta_timestamp, delta_time)
		self.last_timestamp = timestamp
		self.last_time = new_time
		self.pf.propagate_alt(delta_timestamp, data.vx, data.vy, data.altd, data.rotZ, data.header.seq)
		# self.pf.propagate(delta_t, data.ax, data.ay, data.az, data.rotX, data.rotY, data.rotZ)
		# self.pf.correct(delta_t, data.vx, data.vy, data.altd, data.magX, data.magY, data.magZ)

	def ar_correct(self, data):
		self.pf.ar_correct(data)


if __name__ == "__main__":
	#start the class
	rospy.init_node("localize")
	# cmd = BasicCommands()
	# controller = DroneController(cmd)
	l = localize()
	run = raw_input("Press any key to run:")

	print "Running"
	l.listener()
