#!/usr/bin/env python

# particlefilter.py
# Ed Kelley
# Senior thesis, 2012-2013

import sys
import time
import random
from math import *
import numpy
#ROS related initializations
import os
import roslib
import rospy
from walkerrandom import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from tf import *
from tf.transformations import *
from tf import TransformerROS

class particlefilter:
	def __init__(self, num_particles=100, vis_noise=10, ultra_noise=100, mag_noise=37, linear_noise=.002, angular_noise=2.3, ar_noise=10, ar_resample_rate=10):
		print("Starting a particle filer with %d particles" % num_particles)
		self.filename = "/home/ekelley/Dropbox/thesis_data/" + time.strftime('%Y_%m_%d_%H_%M_%S') #in the format YYYYMMDDHHMMSS
		self.fp = open(self.filename + ".txt", "w")
		self.fp_part = open(self.filename+"_part.txt", "w")
		self.fp_ar = open(self.filename+"_ar.txt", "w")
		self.num_particles = num_particles


		self.vis_noise = vis_noise
		self.ultra_noise = ultra_noise
		self.mag_noise = mag_noise
		self.linear_noise = linear_noise
		self.angular_noise = angular_noise
		self.ar_noise = ar_noise

		self.ar_resample_rate = ar_resample_rate


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
		self.ar_est = particle(self) #Estimation using vis odometry and ultrasound
		for i in range(num_particles):
			self.particle_list.append(particle(self))

		self.fp.write("self.step,delta_t,x_acc,y_acc,z_acc,gyr_theta_est,rotX,rotY,delta_theta,self.acc_est.x,self.acc_est.y,self.acc_est.z,self.acc_est.theta,x_vel,y_vel,z_est,magX,magY,magZ,mag_theta_est,new_x,new_y,self.vis_est.x,self.vis_est.y,self.est.x,self.est.y,self.est.theta\n")
		self.est_pose = Point()
		self.line = Marker()
		self.est_pub = rospy.Publisher('pf_pose', Point)
		self.listener = TransformListener()
		self.transformer = TransformerROS()
		self.update_marker()

	#Propogate particles based on accelerometer data and gyroscope-based theta
	def propogate(self, delta_t, x_acc, y_acc, z_acc, rotX, rotY, rotZ):
		if (self.first_propogate):
			self.start_gyr_heading = rotZ

		#PROPOGATE USING THE AVERAGE OF EST.THETA AND THETA_EST-PREV_THETA

		delta_theta = self.clamp_angle((rotZ- self.start_gyr_heading) - self.acc_est.theta) #Should I be using self.est.theta instead of prev_theta?

		for particle in self.particle_list:
			particle.propogate(delta_t, self.convert_g(x_acc), self.convert_g(y_acc), self.convert_g(z_acc), rotX, rotY, delta_theta, True)

		#Propogate estimate based on magnetometer. for testing
		self.acc_est.propogate(delta_t, self.convert_g(x_acc), self.convert_g(y_acc), self.convert_g(z_acc), rotX, rotY, delta_theta, False)

		self.fp.write("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f," % (self.step, delta_t, x_acc, y_acc, z_acc, rotZ, rotX, rotY, delta_theta, self.acc_est.x, self.acc_est.y, self.acc_est.z, self.acc_est.theta))

	def convert_g(self, acc):
		g_to_mmss = 9806.65
		return acc*g_to_mmss


	#Correct particles based on visual odometry and magnetometer readings
	def correct(self, delta_t, x_vel, y_vel, z_est, magX, magY, magZ):
		if (self.first_correct):
			self.start_mag_heading = self.get_heading(magX, magY, magZ)

		mag_theta_est = self.clamp_angle(self.get_heading(magX, magY, magY)-self.start_mag_heading)
		x_delta = (x_vel*cos(radians(mag_theta_est)) - y_vel*sin(radians(mag_theta_est)))*delta_t
		y_delta = (x_vel*sin(radians(mag_theta_est)) + y_vel*cos(radians(mag_theta_est)))*delta_t

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

		self.vis_est.x += x_delta
		self.vis_est.y += y_delta
		self.vis_est.z = z_est
		self.vis_est.theta = mag_theta_est

		self.fp.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (x_vel, y_vel, z_est, magX, magY, magZ, mag_theta_est, new_x, new_y, self.vis_est.x, self.vis_est.y, self.est.x, self.est.y, self.est.theta))
		self.step += 1
		self.estimate()
		self.update_marker()

	def ar_correct(self, marker):
		marker_id = marker.id
		pose = marker.pose

		pose_trans = ()
		pose_rot = ()
		#Is this necessary
		pose_trans = (pose.position.x, pose.position.y, pose.position.z)
		pose_rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

		marker_name = "/marker_%d" % marker_id

		#Get the offset of the marker from the origin
		try:
			(marker_trans,marker_rot) = self.listener.lookupTransform('world', marker_name, rospy.Time(0))
		except (LookupException, ConnectivityException, ExtrapolationException):
			print("Unable to find marker transform")
			return

		try:
			(base_trans,base_rot) = self.listener.lookupTransform('/ardrone/ardrone_base_link', '/ardrone/ardrone_base_bottomcam', rospy.Time(0))
		except (LookupException, ConnectivityException, ExtrapolationException):
			print("Unable to find ardrone transform")
			return

		#Everything is in m not mm
		pose_mat = numpy.matrix(self.transformer.fromTranslationRotation(pose_trans, pose_rot))
		pose_mat_inv = pose_mat.getI()
		marker_mat = numpy.matrix(self.transformer.fromTranslationRotation(marker_trans, marker_rot))
		marker_mat_inv = marker_mat.getI()
		base_mat = numpy.matrix(self.transformer.fromTranslationRotation(base_trans, base_rot))
		base_mat_inv = base_mat.getI()

		origin = numpy.matrix([[0], [0], [0], [1]])

		global_mat = marker_mat * pose_mat_inv * base_mat_inv

		global_trans = translation_from_matrix(global_mat)
		global_rot = rotation_from_matrix(global_mat)

		global_trans = global_trans*1000 #Convert to mm
		global_heading = self.clamp_angle(global_rot.item(0)*180/pi) #Convert to degrees

		self.tag_est.x = global_trans.item(0)
		self.tag_est.y = global_trans.item(1)
		self.tag_est.z = global_trans.item(2)
		self.tag_est.theta = global_rot.item(0)

		#Weight particles
		self.weight_particles(delta_t, self.tag_est.x, self.tag_est.y, self.tag_est.z, self.tag_est.theta, True)

		#Create new set of particles
		self.particle_list = []

		if (ar_resample):
			#Initialize the random selector. Can select items in O(1) time
			wrand = walkerrandom(self.weight_dict.values(), self.weight_dict.keys())

			for i in range(self.num_particles):
				#Create particle from new value
				particle = Particle()
				if (random.random() < self.ar_resample/100):
					particle = Particle(self, self.tag_est.x, self.tag_est.y, self.tag_est.z, self.tag_est.theta)
				#Randomly pick existing particle
				else:
					particle = wrand.random()
				self.particle_list.append(particle)

		
		self.fp_ar.write("%d,%d,%f,%f,%f\n" % (self.step, marker_id, self.tag_est.x, self.tag_est.y, self.tag_est.z, self.tag_est.theta))
		


	def update_marker(self):
		self.est_pose.x = self.est.x
		self.est_pose.y = self.est.y
		self.est_pose.z = self.est.z
		self.est_pub.publish(self.est_pose)


	#Calculate the weight for particles
	def weight_particles(self, delta_t, x_est, y_est, z_est, theta_est, ar=False):
		self.weight_dict = dict()
		weight_sum = 0
		for particle in self.particle_list:

			#Calculate distances
			lat_dist = ((x_est - particle.x)**2 + (y_est - particle.y)**2)**.5
			vert_dist = abs(z_est - particle.z)
			theta_dist = abs(theta_est - particle.theta)

			lat_noise = self.vis_noise
			vert_noise = self.ultra_noise
			theta_noise = self.mag_noise

			#If this module is being called with ar_tag data, use ar_noise
			if (ar):
				lat_noise = self.ar_noise
				vert_noise = self.ar_noise
				theta_noise = self.ar_noise

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

	#Adapted from:
	#http://stackoverflow.com/questions/12412895/calculate-probability-in-normal-distribution-given-mean-std-in-python
	def normpdf(self, x, mean, sd):
	    var = float(sd)**2
	    denom = (2*pi*var)**.5
	    num = exp(-(float(x)-float(mean))**2/(2*var))
	    return num/denom

	# Adapted from:
	# http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
	def get_heading(self, magX, magY, magZ):
		
		heading = (atan2(magX, magY)/pi)*180
		return self.clamp_angle(heading - self.start_mag_heading)

	#Keep angle between -180 and 180
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

	#Update the values of the particle based on acc and gyr data
	def propogate(self, delta_t, x_acc, y_acc, z_acc, rotX, rotY, theta_delta, noise):
		if (self.parent.step%100 == 0):
			self.parent.fp_part.write("%d, %f, %f, %f, %f, %f, %f, %f, %f\n" % (self.parent.step, delta_t, self.x, self.y, self.z, self.x_vel, self.y_vel, self.z_vel, self.theta))
		
		x_acc_noise = x_acc
		y_acc_noise = y_acc
		z_acc_noise = z_acc
		theta_delta_noise = theta_delta
		rotX_noise = rotX
		rotY_noise = rotY

		if (noise):
			x_acc_noise = random.normalvariate(x_acc, self.parent.linear_noise)
			y_acc_noise = random.normalvariate(y_acc, self.parent.linear_noise)
			z_acc_noise = random.normalvariate(z_acc, self.parent.linear_noise)
			rotX_noise = random.normalvariate(rotX, self.parent.angular_noise)
			rotY_noise = random.normalvariate(rotY, self.parent.angular_noise)
			theta_delta_noise = random.normalvariate(theta_delta, self.parent.angular_noise)

		self.theta = self.parent.clamp_angle(self.theta + theta_delta_noise)

		acc_m = numpy.matrix([[x_acc_noise], [y_acc_noise], [z_acc_noise], [1]])

		acc_global_m = rotate(acc_m, rotX_noise, rotY_noise, self.theta)

		x_acc_global = acc_global_m.item(0)
		y_acc_global = acc_global_m.item(1)
		z_acc_global = acc_global_m.item(2) - 0.942871 #From sensor data

		self.x_vel = x_acc_global*delta_t + self.x_vel
		self.y_vel = y_acc_global*delta_t + self.y_vel
		self.z_vel = z_acc_global*delta_t + self.z_vel


		x_delta = self.x_vel*delta_t
		y_delta = self.y_vel*delta_t
		z_delta = self.z_vel*delta_t

		self.x += x_delta
		self.y += y_delta
		self.z += z_delta

	def to_string(self):
		return "(%.2f, %.2f, %.2f, %.4f)" % (self.x, self.y, self.z, self.theta)


#Rotate to global coordinates
def rotate(m, rotX, rotY, rotZ):
	rotX_m = numpy.matrix([[ 1, 0, 0, 0],
							[ 0, cos(radians(rotX)), -sin(radians(rotX)), 0],
							[ 0, sin(radians(rotX)), cos(radians(rotX)), 0],
							[ 0, 0, 0, 1]])
	rotY_m = numpy.matrix([[cos(radians(rotY)), 0, -sin(radians(rotY)), 0],
							[ 0, 1, 0, 0],
							[ sin(radians(rotY)), 0, cos(radians(rotY)), 0],
							[ 0, 0, 0, 1]])
	rotZ_m = numpy.matrix([[cos(radians(rotZ)), -sin(radians(rotZ)), 0, 0],
							[sin(radians(rotZ)),cos(radians(rotZ)),0,0],
							[0,0,1,0],
							[0,0,0,1]])
	return rotZ_m*rotY_m*rotX_m*m;


#Testing basic functionality
def main():
	pf = particlefilter(num_particles=10)

	print "---------INIT---------"

	pf.print_particles()
	pf.propogate(.1, 10, 10, 0, 0, 0, 32, True) 

	print "---------PROP---------"

	pf.print_particles()
	pf.correct(.1, 20, 20, 0, 24, 53, 10)

	print "---------CORRECT---------"

	pf.print_particles()

if __name__ == "__main__":
	main()

