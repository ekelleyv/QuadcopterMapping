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
	def __init__(self, num_particles=1000, linear_noise=.1, angular_noise=.1):
		self.filename = filename= "/home/ekelley/Dropbox/thesis_data/" + time.strftime('%Y_%m_%d_%H_%M_%S') + ".txt" #in the format YYYYMMDDHHMMSS
		self.num_particles = num_particles
		self.linear_noise = linear_noise
		self.angular_noise = angular_noise
		self.particle_list = []
		for i in range(num_particles):
			self.particle_list.append(particle(self))

	#Propogate particles based on accelerometer data and gyroscope-based theta
	def propogate(self, delta_t, x_acc, y_acc, z_acc, theta_est):
		for particle in self.particle_list:
			particle.propogate(delta_t, x_acc, y_acc, z_acc, theta_est) #UPDATE VALUES

	#Correct particles based on visual odometry and magnetometer readings
	def correct(self, delta_t, x_vel, y_vel, z_est, magX, magY, magZ):
		theta_est = 0 #FIGURE OUT HOW TO COMBINE 3D MAG READINGS INTO HEADING
		x_delta = (x_vel*math.cos(theta_est) - y_vel*math.sin(theta_est))*delta_t
		y_delta = (x_vel*math.sin(theta_est) - y_vel*math.cos(theta_est))*delta_t
		self.weight(x_vel, y_vel, )

	#Calculate the weight for particles
	def weight(self):
		pass


	#Normalize all the weights such that they sum to 1
	def normalize(self):
		total = 0
		for particle in self.particle_list:
			total += particle.weight
		for particle in self.particle_list:
			if (total != 0)
			particle.weight = particle.weight/total

	
	#Return an estimate of the pose
	def estimate(self):
		est = self.particle_list[0] #Temporarily return a single particle
		return est

	def print_particles(self):
		for particle in self.particle_list:
			print particle.to_string()


class particle:
	def __init__(self, particlefilter, x=0, y=0, z=0, theta=0):
		self.x = x
		self.y = y
		self.z = z
		self.x_vel = 0;
		self.y_vel = 0;
		self.z_vel = 0;
		self.theta = theta
		self.weight = 1 #How should this be initialized?
		self.parent = particlefilter

	#Update the values of the particle based 
	def propogate(self, delta_t, x_acc, y_acc, z_acc, theta_est):
		x_acc_noise = random.normalvariate(x_acc, self.parent.linear_noise)
		y_acc_noise = random.normalvariate(y_acc, self.parent.linear_noise)
		z_acc_noise = random.normalvariate(z_acc, self.parent.linear_noise)

		self.x = .5*x_acc_noise*delta_t**2 + self.x_vel*delta_t + self.x
		self.y = .5*y_acc_noise*delta_t**2 + self.y_vel*delta_t + self.y
		self.z = .5*z_acc_noise*delta_t**2 + self.z_vel*delta_t + self.z

		self.x_vel = x_acc*delta_t
		self.y_vel = y_acc*delta_t
		self.z_vel = z_acc*delta_t

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

