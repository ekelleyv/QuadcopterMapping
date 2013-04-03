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
	def __init__(self, num_particles=1000, linear_noise=10, angular_noise=.5):
		self.filename = "/home/ekelley/Dropbox/thesis_data/" + time.strftime('%Y_%m_%d_%H_%M_%S') #in the format YYYYMMDDHHMMSS
		self.fp = open(self.filename + ".txt", "w")
		self.fp_part = open(self.filename+"_part.txt", "w")
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
		self.step = 0
		self.est = particle(self)
		for i in range(num_particles):
			self.particle_list.append(particle(self))
		self.fp.write("step,delta_t,x_acc,y_acc,z_acc,theta_est_gyr,norm_delta_theta,prev_theta,x_vel,y_vel,z_est,magX,magY,magZ,theta_est_mag,correct_x,correct_y\n")
		self.fp_part.write("step,delta_t,x,y,z,x_vel,y_vel,z_vel,theta\n")



	#Propogate particles based on accelerometer data and gyroscope-based theta
	def propogate(self, delta_t, x_acc, y_acc, z_acc, theta_est):
		#Dont need to zero gyr
		if (self.first_propogate):
			self.start_gyr_heading = theta_est

		#PROPOGATE USING THE AVERAGE OF EST.THETA AND THETA_EST-PREV_THETA

		norm_delta_theta = self.clamp_angle(theta_est - self.prev_theta - self.start_gyr_heading) #Should I be using self.est.theta instead of prev_theta?

		for particle in self.particle_list:
			particle.propogate(delta_t, convert_g(x_acc), convert_g(y_acc), convert_g(z_acc), norm_delta_theta)

		self.prev_theta = self.clamp_angle(self.prev_theta + norm_delta_theta)

		self.fp.write("%d,%f,%f,%f,%f,%f,%f,%f, " % (self.step, delta_t, x_acc, y_acc, z_acc, theta_est, norm_delta_theta, self.prev_theta))

	def convert_g(acc):
		rate = 9806.65
		return acc*rate


	#Correct particles based on visual odometry and magnetometer readings
	def correct(self, delta_t, x_vel, y_vel, z_est, magX, magY, magZ):
		if (self.first_correct):
			self.start_mag_heading = self.get_heading(magX, magY, magZ)

		theta_est = self.clamp_angle(self.get_heading(magX, magY, magY)-self.start_mag_heading)
		x_delta = (x_vel*math.cos(theta_est) - y_vel*math.sin(theta_est))*delta_t
		y_delta = (x_vel*math.sin(theta_est) + y_vel*math.cos(theta_est))*delta_t

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

		self.fp.write("%f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (x_vel, y_vel, z_est, magX, magY, magZ, theta_est, new_x, new_y))
		self.step += 1


	#Calculate the weight for particles
	def weight_particles(self, delta_t, x_est, y_est, z_est, theta_est):
		self.weight_dict = dict()
		weight_sum = 0
		for particle in self.particle_list:
			#THIS WEIGHTING IS JUST A PLACEHOLDER
			#REPLACE WITH SENSOR MODEL DATA
			x_diff = abs(x_est - particle.x)
			y_diff = abs(y_est - particle.y)
			z_diff = abs(z_est - particle.z)
			theta_diff = abs(theta_est - particle.theta)
			weight = 1/(x_diff + y_diff + z_diff + 1)
			self.weight_dict[particle] = weight
			weight_sum += weight

		#Normalize the weights
		for particle, weight in self.weight_dict.iteritems():
			if (weight_sum != 0):
				weight = weight/weight_sum


	# http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
	def get_heading(self, magX, magY, magZ):
		
		heading = (math.atan2(magX, magY)/math.pi)*180
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
		self.est = particle(self)
		for part, weight in self.weight_dict.iteritems():
			self.est.x += part.x*weight
			self.est.y += part.y*weight
			self.est.z += part.z*weight
			self.est.theta += part.z*weight


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
		if (self.parent.step%100 == 0):
			self.parent.fp_part.write("%d, %f, %f, %f, %f, %f, %f, %f, %f\n" % (self.parent.step, delta_t, self.x, self.y, self.z, self.x_vel, self.y_vel, self.z_vel, self.theta))
		x_acc_noise = random.normalvariate(x_acc, self.parent.linear_noise)
		y_acc_noise = random.normalvariate(y_acc, self.parent.linear_noise)
		z_acc_noise = random.normalvariate(z_acc, self.parent.linear_noise)

		theta_delta_noise = random.normalvariate(theta_delta, self.parent.angular_noise)

		#In local coordinate frame
		self.x_vel = x_acc_noise*delta_t + self.x_vel
		self.y_vel = y_acc_noise*delta_t + self.y_vel
		self.z_vel = z_acc_noise*delta_t + self.z_vel


		x_delta = (self.x_vel*math.cos(self.theta) - self.y_vel*math.sin(self.theta))*delta_t #Measurements in global coordinate frame
		y_delta = (self.x_vel*math.sin(self.theta) + self.y_vel*math.cos(self.theta))*delta_t
		z_delta = self.z_vel*delta_t

		self.x += x_delta
		self.y += y_delta
		self.z += z_delta

		self.theta = self.parent.clamp_angle(self.theta + theta_delta_noise)

	def to_string(self):
		return "(%.2f, %.2f, %.2f, %.4f)" % (self.x, self.y, self.z, self.theta)


def main():
	pf = particlefilter(num_particles=10)

	print "---------INIT---------"

	pf.print_particles()
	pf.propogate(.1, 10, 10, 0, 32)

	print "---------PROP---------"

	pf.print_particles()
	pf.correct(.1, 20, 20, 0, 24, 53, 10)

	print "---------CORRECT---------"

	pf.print_particles()

if __name__ == "__main__":
	main()

