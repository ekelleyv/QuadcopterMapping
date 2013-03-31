#!/usr/bin/env python
 
import sys
import time
import random
import math
#ROS related initializations
import os
from walkerrandom import *

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
	def __init__(self, num_particles=1000, linear_noise=10, angular_noise=5):
		self.filename = filename= "/home/ekelley/Dropbox/thesis_data/" + time.strftime('%Y_%m_%d_%H_%M_%S') + ".txt" #in the format YYYYMMDDHHMMSS
		self.num_particles = num_particles
		self.linear_noise = linear_noise
		self.angular_noise = angular_noise
		self.start_mag_heading = 0
		self.start_gyr_heading = 0
		self.prev_theta = 0
		self.particle_list = []
		self.weight_dict = dict()
		self.first_propogate = True
		self.first_correct = True
		self.est = particle()
		for i in range(num_particles):
			self.particle_list.append(particle(self))



	#Propogate particles based on accelerometer data and gyroscope-based theta
	def propogate(self, delta_t, x_acc, y_acc, z_acc, theta_est):
		if (self.first_propogate):
			self.start_gyr_heading = theta_est

		norm_delta_theta = self.clamp_angle(theta_est - self.prev_theta - self.start_gyr_heading)
		for particle in self.particle_list:
			particle.propogate(delta_t, x_acc, y_acc, z_acc, norm_delta_theta)

		self.prev_theta = self.clamp_angle(self.prev_theta + norm_delta_theta)



	#Correct particles based on visual odometry and magnetometer readings
	def correct(self, delta_t, x_vel, y_vel, z_est, magX, magY, magZ):
		if (self.first_correct):
			self.start_mag_heading = self.get_heading(magX, magY, magZ)

		theta_est = self.get_heading(magX, magY, magY)
		x_delta = (x_vel*math.cos(theta_est) - y_vel*math.sin(theta_est))*delta_t
		y_delta = (x_vel*math.sin(theta_est) - y_vel*math.cos(theta_est))*delta_t

		new_x = x_delta + self.est.x
		new_y = y_delta + self.est.y

		#Weight particles
		self.weight_particles(delta_t, new_x, new_y, z_est, theta_est)

		#Create new set of particles
		self.particle_list = []

		#Initialize the random selector. Can select items in O(1) time
		wrand = walkerrandom(self.weight_dict.values(), self.weight_dict.keys())

		for i in range(self.num_particles):
			particle = wrand.random()
			self.particle_list.append(particle)

		self.est = self.estimate()


	#Calculate the weight for particles
	def weight_particles(self, delta_t, x_est, y_est, z_est, theta_est):
		self.weight_dict = dict()
		weight_sum = 0
		for particle in self.particle_list:
			weight = 1 #Some function of difference between particle location and the estimated position based on sensors
			self.weight_dict[particle] = weight
			weight_sum += weight

		#Normalize the weights
		for particle, weight in self.weight_dict:
			if (weight_sum != 0):
				weight = weight/weight_sum


	# http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
	def get_heading(self, magX, magY, magZ):
		heading = 0
		if (magY > 0):
			heading = 90 - math.atan(magX/magY)*180
		elif (magY <0):
			heading = 270 - math,atan(magX/magY)*180
		elif ((y==0) and (x < 0)):
			heading = 180
		else:
			heading = 0

		return self.clamp_angle(heading - self.start_mag_heading)



	def clamp_angle(self, angle):
		if (angle > 180):
			return angle - 360
		elif (angle < -180):
			return angle + 360
		else:
			return angle

	
	#Return an estimate of the pose
	#For now just use linear combination. Should we cluster instead?
	def estimate(self):
		est = particle()
		for particle, weight in self.weight_dict:
			est.x += particle.x*weight
			est.y += particle.y*weight
			est.z += particle.z*weight
			est.theta += particle.z*weight

		return est


	def print_particles(self):
		for particle in self.particle_list:
			print particle.to_string()


class particle:
	def __init__(self, particlefilter, x=0, y=0, z=0, theta=0):
		self.x = x #Global
		self.y = y
		self.z = z
		self.x_vel = 0; #Local
		self.y_vel = 0;
		self.z_vel = 0;
		self.theta = theta
		self.parent = particlefilter

	#Update the values of the particle based 
	def propogate(self, delta_t, x_acc, y_acc, z_acc, theta_delta):
		x_acc_noise = random.normalvariate(x_acc, self.parent.linear_noise)
		y_acc_noise = random.normalvariate(y_acc, self.parent.linear_noise)
		z_acc_noise = random.normalvariate(z_acc, self.parent.linear_noise)

		theta_delta_noise = random.normalvariate(theta_delta, self.parent.angular_noise)

		#In local coordinate frame
		self.x_vel = x_acc_noise*delta_t + self.x_vel
		self.y_vel = y_acc_noise*delta_t + self.y_vel
		self.z_vel = z_acc_noise*delta_t + self.z_vel


		x_delta = (x_vel*math.cos(theta_est) - y_vel*math.sin(theta_est))*delta_t #Measurements in global coordinate frame
		y_delta = (x_vel*math.sin(theta_est) - y_vel*math.cos(theta_est))*delta_t
		z_delta = self.z_vel*delta_t

		self.x += x_delta
		self.y += y_delta
		self.z += z_delta

		self.theta = self.parent.clamp_angle(self.theta + theta_delta)

	def to_string(self):
		return "(%.2f, %.2f, %.2f, %.4f)" % (self.x, self.y, self.z, self.theta)


def main():
	pf = particlefilter(num_particles=10)
	pf.print_particles()
	pf.set_heading(30, 24, 50, 10)
	pf.propogate(.1, 10, 10, 0, 32)

	print "------------------"

	pf.print_particles()
	pf.correct(.1, 20, 20, 0, 24, 53, 10)

	print "------------------"

	pf.print_particles()

if __name__ == "__main__":
	main()

