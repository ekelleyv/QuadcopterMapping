#!/usr/bin/env python
 
import sys
import time
import random
import math
#ROS related initializations
import roslib
import rospy
import os
from std_msgs.msg import String, Empty

#Coordinate frame info
# -linear.x: move backward
# +linear.x: move forward
# -linear.y: move right
# +linear.y: move left
# -linear.z: move down
# +linear.z: move up

# -angular.z: turn left
# +angular.z: turn right
class particlefilter:
	def __init__(self, num_particles=1000, linear_noise=.1, angular_noise=.01):
		self.filename = filename= "/home/ekelley/Dropbox/thesis_data/" + time.strftime('%Y_%m_%d_%H_%M_%S') + ".txt" #in the format YYYYMMDDHHMMSS
		self.num_particles = num_particles
		self.linear_noise = linear_noise
		self.angular_noise = angular_noise
		self.particle_list = []
		for i in range(num_particles):
			self.particle_list.append(particle())

	def propogate(self, delta_t, x_vel, y_vel, z_est, theta_est):
		x_delta = (x_vel*math.cos(theta_est) - y_vel*math.sin(theta_est))*delta_t
		y_delta = (x_vel*math.sin(theta_est) - y_vel*math.cos(theta_est))*delta_t
		for particle in self.particle_list:
			particle.propogate(delta_t, x_delta, y_delta, z_est, theta_est)

	#Eventually this should use camera
	def correct(self, magX, magY, magZ):
		return

	def estimate(self):
		est = self.particle_list[0]
		return est
		# for particle in self.particle_list:

	def print_particles(self):
		for particle in self.particle_list:
			print particle.to_string()


class particle:
	def __init__(self, x=0, y=0, z=0, theta=0):
		self.x = x
		self.y = y
		self.z = z
		self.theta = theta

	def propogate(self, delta_t, x_delta, y_delta, z_est, theta_est):
		self.x += x_delta
		self.y += y_delta
		self.z = z_est
		self.theta = theta_est

	def to_string(self):
		return "(%.2f, %.2f, %.2f, %.4f)" % (self.x, self.y, self.z, self.theta)

if __name__ == "__main__":
	#start the class
	my_particles = particles(num_particles=10)
	my_particles.print_particles()
	print("========================")
	my_particles.propogate(10, 1, 1, 1, 0)
	my_particles.print_particles()

