#!/usr/bin/env python
 
import sys
import time
import random
import math
#ROS related initializations
import os
import rospy
from walkerrandom import *
from std_msgs.msg import *
from ardrone_autonomy.msg import Navdata
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose

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
	def __init__(self, num_particles=10, vis_noise=10, ultra_noise=100, mag_noise=37, linear_noise=.002, angular_noise=2.3):
		self.filename = "/home/ekelley/Dropbox/thesis_data/" + time.strftime('%Y_%m_%d_%H_%M_%S') #in the format YYYYMMDDHHMMSS
		self.fp = open(self.filename + ".txt", "w")
		self.fp_part = open(self.filename+"_part.txt", "w")
		self.num_particles = num_particles


		self.vis_noise = vis_noise
		self.ultra_noise = ultra_noise
		self.mag_noise = mag_noise
		self.linear_noise = linear_noise
		self.angular_noise = angular_noise


		self.start_mag_heading = 0
		self.start_gyr_heading = 0
		self.gyr_theta = 0
		self.particle_list = []
		self.weight_dict = dict()
		self.first_propogate = True
		self.first_correct = True
		self.step = 0
		self.est = particle(self)
		self.acc_est = particle(self) #Estimation using acc and gyr
		self.vis_est = particle(self) #Estimation using vis odometry and ultrasound
		for i in range(num_particles):
			self.particle_list.append(particle(self))
		#self.acc_est.x, self.acc_est.y, self.acc_est.z, self.acc_est.theta))
		self.fp.write("self.step,delta_t,x_acc,y_acc,z_acc,gyr_theta_est,delta_theta,self.acc_est.x,self.acc_est.y,self.acc_est.z,self.acc_est.theta,x_vel,y_vel,z_est,magX,magY,magZ,mag_theta_est,new_x,new_y,self.vis_est.x,self.vis_est.y\n")
		self.est_marker = Marker()
		self.update_marker()

	#Propogate particles based on accelerometer data and gyroscope-based theta
	def propogate(self, delta_t, x_acc, y_acc, z_acc, gyr_theta_est):
		if (self.first_propogate):
			self.start_gyr_heading = gyr_theta_est

		#PROPOGATE USING THE AVERAGE OF EST.THETA AND THETA_EST-PREV_THETA

		delta_theta = self.clamp_angle((gyr_theta_est- self.start_gyr_heading) - self.acc_est.theta) #Should I be using self.est.theta instead of prev_theta?

		for particle in self.particle_list:
			particle.propogate(delta_t, self.convert_g(x_acc), self.convert_g(y_acc), self.convert_g(z_acc), delta_theta, True)

		#Propogate estimate based on magnetometer. for testing
		self.acc_est.propogate(delta_t, self.convert_g(x_acc), self.convert_g(y_acc), self.convert_g(z_acc), delta_theta, False)

		self.fp.write("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f, " % (self.step, delta_t, x_acc, y_acc, z_acc, gyr_theta_est, delta_theta, self.acc_est.x, self.acc_est.y, self.acc_est.z, self.acc_est.theta))
		self.update_marker()

	def convert_g(self, acc):
		g_to_mmss = 9806.65
		return acc*g_to_mmss


	#Correct particles based on visual odometry and magnetometer readings
	def correct(self, delta_t, x_vel, y_vel, z_est, magX, magY, magZ):
		if (self.first_correct):
			self.start_mag_heading = self.get_heading(magX, magY, magZ)

		mag_theta_est = self.clamp_angle(self.get_heading(magX, magY, magY)-self.start_mag_heading)
		x_delta = (x_vel*math.cos(mag_theta_est) - y_vel*math.sin(mag_theta_est))*delta_t
		y_delta = (x_vel*math.sin(mag_theta_est) + y_vel*math.cos(mag_theta_est))*delta_t

		new_x = x_delta + self.est.x
		new_y = y_delta + self.est.y

		#Weight particles
		self.weight_particles(delta_t, new_x, new_y, z_est, mag_theta_est)

		#Create new set of particles
		self.particle_list = []

		#Initialize the random selector. Can select items in O(1) time
		wrand = walkerrandom(self.weight_dict.values(), self.weight_dict.keys())

		for i in range(self.num_particles):
			particle = wrand.random()
			self.particle_list.append(particle)

		self.vis_est.x += x_delta;
		self.vis_est.y += y_delta;
		self.vis_est.z = z_est;
		self.vis_est.theta = mag_theta_est

		self.fp.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (x_vel, y_vel, z_est, magX, magY, magZ, mag_theta_est, new_x, new_y, self.vis_est.x, self.vis_est.y))
		self.step += 1
		self.estimate()
		self.update_marker()
		

	def update_marker(self):
		# self.est_marker.stamp = rospy.Time.now()
		self.est_marker.header.frame_id = "/ardrone_base_link"
		self.est_marker.header.stamp = rospy.Time.now()
		self.est_marker.type = Marker.SPHERE
		self.est_marker.pose.position.x = self.est.x
		self.est_marker.pose.position.y = self.est.y
		self.est_marker.pose.position.z = self.est.z

		self.est_marker.pose.orientation.x = 0
		self.est_marker.pose.orientation.y = 0
		self.est_marker.pose.orientation.z = 0
		self.est_marker.pose.orientation.w = 0

		self.est_marker.scale.x = .1
		self.est_marker.scale.y = .1
		self.est_marker.scale.z = .1

		self.est_marker.color.a = 1
		self.est_marker.color.r = 0
		self.est_marker.color.g = 1
		self.est_marker.color.b = 0


	#Calculate the weight for particles
	def weight_particles(self, delta_t, x_est, y_est, z_est, theta_est):
		self.weight_dict = dict()
		weight_sum = 0
		for particle in self.particle_list:
			#THIS WEIGHTING IS JUST A PLACEHOLDER
			#REPLACE WITH SENSOR MODEL DATA

			#Calculate distances
			lat_dist = ((x_est - particle.x)**2 + (y_est - particle.y)**2)**.5
			vert_dist = abs(z_est - particle.z)
			theta_dist = abs(theta_est - particle.theta)

			lat_weight = self.normpdf(lat_dist, 0, self.vis_noise) #Potentially do the z distance separately?
			vert_weight = self.normpdf(vert_dist, 0, self.ultra_noise)
			theta_weight = self.normpdf(theta_dist, 0, self.mag_noise)

			weight = lat_weight + vert_weight + theta_weight

			self.weight_dict[particle] = weight
			weight_sum += weight

		#Normalize the weights
		for particle, weight in self.weight_dict.iteritems():
			if (weight_sum != 0):
				weight = weight/weight_sum

	#http://stackoverflow.com/questions/12412895/calculate-probability-in-normal-distribution-given-mean-std-in-python
	def normpdf(self, x, mean, sd):
	    var = float(sd)**2
	    denom = (2*math.pi*var)**.5
	    num = math.exp(-(float(x)-float(mean))**2/(2*var))
	    return num/denom

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
	def propogate(self, delta_t, x_acc, y_acc, z_acc, theta_delta, noise):
		if (self.parent.step%100 == 0):
			self.parent.fp_part.write("%d, %f, %f, %f, %f, %f, %f, %f, %f\n" % (self.parent.step, delta_t, self.x, self.y, self.z, self.x_vel, self.y_vel, self.z_vel, self.theta))
		
		x_acc_noise = x_acc
		y_acc_noise = y_acc
		z_acc_noise = z_acc
		theta_delta_noise = theta_delta

		if (noise):
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

