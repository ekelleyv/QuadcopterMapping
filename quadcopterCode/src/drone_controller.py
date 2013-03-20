#!/usr/bin/env python

#What of this can I get rid of?

import roslib; roslib.load_manifest('quadcopterCode')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
from drone_status import DroneStatus
from basic_commands import BasicCommands
from waypoints import waypoints

CONTROLLER_PERIOD = 100 #ms

LINEAR_


class DroneController(BasicCommands):
	def __init__(self):
		self.x_est = 0
		self.y_est = 0
		self.z_est = 0
		self.theta_est = 0

		self.waypoints = waypoints

		#This is probably not how this is actually done
		self.pos_sub = rospy.Subscriber('/localize/tf', tf, update_position)

		self.controller_timer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.update_command)

	def update_position(self, position):
		#some magic happens and I update the position values in the controller

	
	def update_command(self):
		current_waypoint = self.waypoints[self.waypoint_index]



