#!/usr/bin/env python

#What of this can I get rid of?

import roslib; roslib.load_manifest('quadcopterCode')
import rospy
import tf

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from visualization_msgs.msg import *

# An enumeration of Drone Statuses
from drone_status import DroneStatus
from basic_commands import BasicCommands
from waypoints import waypoints
from localize import *
from particlefilter import *

from math import *

CONTROLLER_PERIOD = 100 #ms

LINEAR_ERROR = 200 #mm

ANGULAR_ERROR = 5 #degrees

LINEAR_MAX = .3 #Max tilt amount (unitless)

ANGULAR_MAX = .2 #Max turn amount (unitless)

LINEAR_GAIN = .1 # Pick good values
ANGULAR_GAIN = .1 #




class DroneController():
	def __init__(self):
		self.cmd = BasicCommands()
		self.localize = localize()
		self.pose = particle(self)
		self.start = True

		self.waypoints = waypoints("/home/ekelley/ros_workspace/sandbox/QuadcopterMapping/quadcopterCode/data/waypoints.txt")

		self.current_waypoint = self.waypoints.get_waypoint()

		# self.controller_timer = rospy.Timer(rospy.Duration(CONTROLLER_PERIOD/1000.0),self.update_command)

	def get_distance(self):
		return sqrt((self.current_waypoint.x - self.pose.x)**2 + (self.current_waypoint.y - self.pose.y)**2 + (self.current_waypoint.y - self.pose.y)**2)

	def get_angle_diff(self):
		return abs(self.clamp_angle(self.current_waypoint.theta - self.pose.theta))

	def clamp_angle(self, angle):
		if (angle > 180):
			return angle - 360
		elif (angle < -180):
			return angle + 360
		else:
			return angle

	def listener(self):
		rospy.Subscriber("/ardrone/navdata", Navdata, self.update_command)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def update_command(self, data):
		self.localize.update(data)
		self.pose = self.localize.estimate()

		distance = self.get_distance()
		angle = self.get_angle_diff()

		#If it hit the target, move on
		if ((distance < LINEAR_ERROR) and (angle < ANGULAR_ERROR)):
			self.current_waypoint = self.waypoints.get_waypoint()

		x_diff = (self.current_waypoint.x - self.pose.x)
		y_diff = (self.current_waypoint.y - self.pose.y)
		z_diff = (self.current_waypoint.z - self.pose.z)

		#Define tilt as being within - and + MAX values

		#SetCommand
		# if (self.start):
		# 	self.cmd.SendTakeoff()
		# 	self.start = false
		# elif (self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering):
		# 	self.cmd.SetCommand(roll=0,pitch=0,yaw_velocity=0,z_velocity=0)

def main():
	rospy.init_node("controller")
	controller = DroneController()
	run = raw_input("Press any key to run:")

	print "Running"
	controller.listener()

if __name__ == '__main__':
	main()


