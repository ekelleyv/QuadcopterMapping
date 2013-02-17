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
class particles:
	def __init__(self, num_particles=1000, initial_z=0, linear_noise=.1, angular_noise=.01):
		self.num_particles = num_particles
		self.linear_noise = linear_noise
		self.angular_noise = angular_noise
		self.particle_list = []
		for i in range(num_particles):
			self.particle_list.append(particle(z=initial_z))

	def propogate(self, delta_t, x_vel, y_vel, z_vel, theta_vel):
		for particle in self.particle_list:
			x_noise = random.gauss(x_vel, self.linear_noise)
			y_noise = random.gauss(y_vel, self.linear_noise)
			z_noise = random.gauss(z_vel, self.linear_noise)
			theta_noise = random.gauss(theta_vel, self.angular_noise)

			x_delta = x_noise*delta_t
			y_delta = y_noise*delta_t
			z_delta = z_noise*delta_t
			theta_delta = theta_noise*delta_t
			particle.propogate(x_delta, y_delta, z_delta, theta_delta)
	
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

	def propogate(self, x_delta, y_delta, z_delta, theta_delta):
		self.x += x_delta
		self.y += y_delta
		self.z += z_delta
		self.theta = (self.theta + theta_delta)%(math.pi*2) #Unit circle, forward is x, left is positive

	def to_string(self):
		return "(%.2f, %.2f, %.2f, %.4f)" % (self.x, self.y, self.z, self.theta)

if __name__ == "__main__":
	#start the class
	my_particles = particles(num_particles=10)
	my_particles.print_particles()
	print("========================")
	my_particles.propogate(10, 1, 1, 1, 0)
	my_particles.print_particles()

