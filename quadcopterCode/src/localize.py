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
		self.buffer_index = 0
		self.buffer_size = 5
		self.vx_buffer = [0]*self.buffer_size
		self.vy_buffer = [0]*self.buffer_size
		self.altd_buffer = [0]*self.buffer_size
		self.rotX_buffer = [0]*self.buffer_size
		self.rotY_buffer = [0]*self.buffer_size
		self.rotZ_buffer = [0]*self.buffer_size
		self.buffer_timestamp = 0
		# Noisy no resample
		# self.pf = particlefilter(num_particles=100, vis_noise = 100, ultra_noise = 100, linear_noise = 0, angular_noise = .01, ar_noise=100, ar_resample_rate=0, ar_resample=False)
		# No noise resample
		# self.pf = particlefilter(num_particles=1, vis_noise = 0, ultra_noise = 0, linear_noise = 0, angular_noise = 0, ar_noise=100, ar_resample_rate=0, ar_resample=True)
		
		#Noisy resample
		self.pf = particlefilter(num_particles=50, vis_noise = 10, ultra_noise = 1, linear_noise = 0, angular_noise = .05, ar_noise=200, ar_resample_rate=15, ar_resample=True)
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

		if (self.buffer_index >= self.buffer_size):
			print "BUFFER INDEX TOO LARGE %d" % self.buffer_index
			self.buffer_index = 0

		self.vx_buffer[self.buffer_index] = data.vx
		self.vy_buffer[self.buffer_index] = data.vy
		self.altd_buffer[self.buffer_index] = data.altd
		self.rotX_buffer[self.buffer_index] = data.rotX
		self.rotY_buffer[self.buffer_index] = data.rotY
		self.rotZ_buffer[self.buffer_index] = data.rotZ


		delta_timestamp = timestamp - self.last_timestamp #Time in seconds
		delta_time = new_time - self.last_time
		self.delay += delta_time - delta_timestamp
		# print "Percent Realtime= %3.0f%% | Total delay: %2.2f | Delta timestamp: %1.6f | Delta time: %1.6f" % ((delta_timestamp/delta_time) * 100, abs(self.delay), delta_timestamp, delta_time)
		self.last_timestamp = timestamp
		self.last_time = new_time

		self.buffer_timestamp += delta_timestamp
		self.buffer_index += 1
		if (self.buffer_index == self.buffer_size):
			vx_avg = numpy.average(self.vx_buffer)
			vy_avg = numpy.average(self.vy_buffer)
			altd_avg = numpy.average(self.altd_buffer)
			rotX_avg = numpy.average(self.rotX_buffer)
			rotY_avg = numpy.average(self.rotY_buffer)
			rotZ_avg = numpy.average(self.rotZ_buffer)
			self.pf.propagate_alt(self.buffer_timestamp, vx_avg, vy_avg, altd_avg, rotX_avg, rotY_avg, rotZ_avg)
			self.buffer_timestamp = 0
			self.buffer_index = 0
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
