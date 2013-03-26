#!/usr/bin/env python

#What of this can I get rid of?

import roslib; roslib.load_manifest('quadcopterCode')
import rospy
import tf

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
from drone_status import DroneStatus
from basic_commands import BasicCommands
from waypoints import waypoints

from math import *

CONTROLLER_PERIOD = 100 #ms

LINEAR_ERROR = 200 #mm

ANGULAR_ERROR = (2*PI)/16.0 #ASSUMING RADIANS

LINEAR_MAX = .3 #Max tilt amount (unitless)

ANGULAR_MAX = .2 #Max turn amount (unitless)

LINEAR_GAIN = .1 # Pick good values
ANGULAR_GAIN = .1 #


class DroneController(BasicCommands):
	def __init__(self):
		self.x_est = 0
		self.y_est = 0
		self.z_est = 0
		self.theta_est = 0

		self.waypoints = waypoints("/home/ekelley/ros_workspace/sandbox/QuadcopterMapping/quadcopterCode/data/waypoints.txt")

		self.current_waypoint = self.waypoints.get_waypoint();

		#This is probably not how this is actually done
		self.pos_sub = rospy.Subscriber('/localize/tf', tf, update_position)

		self.controller_timer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.update_command)

	def get_distance(self):
		return sqrt((self.current_waypoint.x - self.x_est)**2 + (self.current_waypoint.z - self.z_est)**2 + (self.current_waypoint.z - self.z_est)**2)

	def get_angle_diff(self):
		return 

	def update_position(self, position):
		#some magic happens and I update the position values in the controller



	def update_command(self):
		distance = self.get_distance()
		angle = self.get_angle_diff()

		#If it hit the target, move on
		if ((distance < LINEAR_ERROR) and (angle < ANGLE_ERROR)):
			self.current_waypoint = self.waypoints.get_waypoint()

		x_diff = (self.current_waypoint.x - self.x_est)
		y_diff = (self.current_waypoint.y - self.y_est)
		z_diff = (self.current_waypoint.z - self.z_est)

		#Define tilt as being within - and + MAX values

		#SetCommand




