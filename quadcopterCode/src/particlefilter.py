#!/usr/bin/env python

# particlefilter.py
# Ed Kelley
# Senior thesis, 2012-2013

import sys
import time
import random
from math import *
from copy import *
import numpy
from utils import *
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
	def __init__(self, num_particles=100, vis_noise=10, ultra_noise=100, mag_noise=37, linear_noise=.002, angular_noise=2.3, ar_noise=10, ar_resample_rate=10, ar_resample=True):
		print("Starting a particle filer with %d particles" % num_particles)
		self.filename = os.path.expanduser("~/Dropbox/thesis_data/" + time.strftime('%Y_%m_%d_%H_%M_%S')) #in the format YYYYMMDDHHMMSS
		self.fp = open(self.filename + ".txt", "w")
		self.fp_part = open(self.filename+"_part.txt", "w")
		self.fp_ar = open(self.filename+"_ar.txt", "w")
		self.fp_alt = open(self.filename+"_alt.txt", "w")
		self.num_particles = num_particles


		self.vis_noise = vis_noise
		self.ultra_noise = ultra_noise
		self.mag_noise = mag_noise
		self.linear_noise = linear_noise
		self.angular_noise = angular_noise
		self.ar_noise = ar_noise

		self.ar_resample_rate = ar_resample_rate
		self.ar_resample = ar_resample

		self.rotY = 0
		self.rotX = 0


		self.start_mag_heading = 0
		self.start_gyr_heading = 0
		self.gyr_theta = 0
		self.particle_list = []
		self.weight_dict = dict()
		self.first_propagate = True
		self.first_correct = True
		self.step = 0
		self.est = particle(self)
		self.acc_est = particle(self) #Estimation using acc and gyr
		self.vis_est = particle(self) #Estimation using vis odometry and ultrasound
		self.tag_est = particle(self) #Estimation using visual tags

		for i in range(num_particles):
			self.particle_list.append(particle(self))

		self.est_pose = Pose()
		self.est_pub = rospy.Publisher('pf_localization', Pose)
		self.listener = TransformListener()
		self.transformer = TransformerROS()
		self.publish_pose()

	def propagate_alt(self, delta_t, x_vel, y_vel, altd, rotX, rotY, rotZ):
		if (self.first_propagate):
			self.start_gyr_heading = rotZ
			self.first_propagate = False

		self.rotX = rotX
		self.rotY = rotY

		# print "Zeroed rotZ: %f" % clamp_angle(rotZ - self.start_gyr_heading)
		delta_theta = clamp_angle((rotZ - self.start_gyr_heading) - self.est.theta)

		self.vis_est.propagate_alt(delta_t, x_vel, y_vel, altd, delta_theta, False)

		for particle in self.particle_list:
			particle.propagate_alt(delta_t, x_vel, y_vel, altd, delta_theta, True)


		#Move this to correct_alt at some point
		self.step += 1
		self.estimate_equal()
		self.publish_pose()
		self.fp_alt.write("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (self.step, delta_t, self.est.x, self.est.y, self.est.z, self.est.theta, self.vis_est.x, self.vis_est.y, self.vis_est.z, self.vis_est.theta))


	def ar_correct(self, marker):
		print "Correcting"
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
		global_heading = clamp_angle(global_rot[0]*180/pi + 180) #Convert to degrees

		self.tag_est.x = global_trans[0]
		self.tag_est.y = global_trans[1]
		self.tag_est.z = global_trans[2]
		self.tag_est.theta = global_heading

		if (self.ar_resample):
			#Weight particles
			self.weight_particles(self.tag_est.x, self.tag_est.y, self.tag_est.z, self.tag_est.theta, True)
			#Create new set of particles
			self.particle_list = []
			
			#Initialize the random selector. Can select items in O(1) time
			wrand = walkerrandom(self.weight_dict.values(), self.weight_dict.keys())

			for i in range(self.num_particles):
				#Create particle from new value
				new_particle = particle(self)
				rand_val = random.random()
				threshold = self.ar_resample_rate/100.0
				if (rand_val < threshold):
					# print "Particle at (%f, %f, %f, %f)" % (self.tag_est.x, self.tag_est.y, self.tag_est.z, self.tag_est.theta)
					new_particle = particle(self, self.tag_est.x, self.tag_est.y, self.tag_est.z, self.tag_est.theta)
				#Randomly pick existing particle
				else:
					# print "Picking existing"
					new_particle = copy(wrand.random())
				self.particle_list.append(new_particle)

		if (len(self.particle_list) != self.num_particles):
			print("WRONG NUMBER OF PARTICLES")

		
		self.fp_ar.write("%d,%d,%f,%f,%f,%f\n" % (self.step, marker_id, self.tag_est.x, self.tag_est.y, self.tag_est.z, self.tag_est.theta))
		


	#Calculate the weight for particles
	def weight_particles(self, x_est, y_est, z_est, theta_est, ar=False):
		# print "Weighting......."
		self.weight_dict = dict()
		# print "Particle List:"
		# for particle in self.particle_list:
		# 	print particle.to_string()

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

			lat_weight = normpdf(lat_dist, 0, lat_noise) #Potentially do the z distance separately?
			vert_weight = normpdf(vert_dist, 0, vert_noise)
			theta_weight = normpdf(theta_dist, 0, theta_noise)

			weight = lat_weight + vert_weight + theta_weight


			self.weight_dict[copy(particle)] = weight
			weight_sum += weight

		# print "Compare to (%f, %f, %f, %f)" % (x_est, y_est, z_est, theta_est)
		#Normalize the weights
		for particle, weight in self.weight_dict.iteritems():
			if (weight_sum != 0):
				self.weight_dict[particle] = weight/weight_sum
			# print "(%s, %f)" % (particle.to_string(), self.weight_dict[particle])

	
	#Return an estimate of the pose
	#For now just use linear combination. Should we cluster instead?
	def estimate(self):
		self.est = particle(self)
		total = 0
		for part, weight in self.weight_dict.iteritems():
			self.est.x += part.x*weight
			self.est.y += part.y*weight
			self.est.z += part.z*weight
			self.est.theta += part.z*weight
			total += weight

		self.est.x = self.est.x/total
		self.est.y = self.est.y/total
		self.est.z = self.est.z/total
		self.est.theta = self.est.theta/total

	def publish_pose(self):
		self.est_pose.position.x = self.est.x
		self.est_pose.position.y = self.est.y
		self.est_pose.position.z = self.est.z

		rad_theta = self.est.theta*(pi/180)

		quat = quaternion_about_axis(rad_theta, (0, 0, 1))

		self.est_pose.orientation.x = quat[0]
		self.est_pose.orientation.y = quat[1]
		self.est_pose.orientation.z = quat[2]
		self.est_pose.orientation.w = quat[3]

		self.est_pub.publish(self.est_pose)


	def estimate_equal(self):
		self.est = particle(self)
		x_val = []
		y_val = []
		z_val = []
		theta_val = []
		total = 0
		for part in self.particle_list:
			self.est.x += part.x
			self.est.y += part.y
			self.est.z += part.z
			self.est.theta += part.theta
			x_val.append(part.x)
			y_val.append(part.y)
			z_val.append(part.z)
			theta_val.append(part.theta)
			total += 1

		self.est.x = self.est.x/total
		self.est.y = self.est.y/total
		self.est.z = self.est.z/total
		self.est.theta = self.est.theta/total

		print "%s : Est" % self.est.to_string()
		# print self.est.theta



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

	def propagate_alt(self, delta_t, x_vel, y_vel, altd, delta_theta, noise=True):
		if (noise):
			self.parent.fp_part.write("%d, %f, %f, %f, %f, %f, %f, %f, %f\n" % (self.parent.step, delta_t, self.x, self.y, self.z, self.theta, x_vel, y_vel, altd))
		vel_m = numpy.matrix([[x_vel], [y_vel], [0], [1]])

		angular_noise = 0
		vis_noise = 0
		ultra_noise = 0

		if (noise):
			angular_noise = self.parent.angular_noise
			vis_noise = self.parent.vis_noise
			ultra_noise = self.parent.ultra_noise

		if (delta_theta > 15):
			delta_theta = 0

		delta_theta_noise = random.normalvariate(delta_theta, angular_noise)
		# print delta_theta_noise
		self.theta = clamp_angle(self.theta + delta_theta_noise)

		vel_global_m = rotate(vel_m, 0, 0, self.theta)

		x_vel_global = vel_global_m.item(0)
		y_vel_global = vel_global_m.item(1)

		x_vel_global_noise = random.normalvariate(x_vel_global, vis_noise)
		y_vel_global_noise = random.normalvariate(y_vel_global, vis_noise)

		# z_noise = random.normalvariate(altd, ultra_noise)
		z_noise = altd

		x_delta = x_vel_global_noise*delta_t
		y_delta = y_vel_global_noise*delta_t

		self.x += x_delta
		self.y += y_delta
		self.z = z_noise

		# if (not noise):
		# 	print "(%f, %f, %f)" % (delta_t, x_vel, y_vel)
		# 	print "%s : Vis" % self.to_string() 

	def to_string(self):
		return "(%.2f, %.2f, %.2f, %.4f)" % (self.x, self.y, self.z, self.theta)


#Testing basic functionality
def main():
	pf = particlefilter(num_particles=10)

	print "---------INIT---------"

	pf.print_particles()
	pf.propagate(.1, 10, 10, 0, 0, 0, 32, True) 

	print "---------PROP---------"

	pf.print_particles()
	pf.correct(.1, 20, 20, 0, 24, 53, 10)

	print "---------CORRECT---------"

	pf.print_particles()

if __name__ == "__main__":
	main()

