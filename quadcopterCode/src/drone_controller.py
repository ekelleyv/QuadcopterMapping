#!/usr/bin/env python

# drone_controller.py
# Ed Kelley
# Senior thesis, 2012-2013

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
from utils import *
from keyboard_controller import *
from drone_video_display import DroneVideoDisplay
from waypoints import waypoints
from localize import *
from particlefilter import *

from math import *
from time import *


LINEAR_ERROR = 200 #mm

ANGULAR_ERROR = 5 #degrees

LINEAR_MAX = .3 #Max tilt amount (unitless)

ANGULAR_MAX = .2 #Max turn amount (unitless)

LINEAR_GAIN = .1 # Pick good values
ANGULAR_GAIN = .1 #




class DroneController(DroneVideoDisplay):
	def __init__(self, cmd = None):
		# self.cmd = BasicCommands()
		self.localize = localize()
		self.pose = particle(self)
		self.start = True
		self.start_time = time()
		self.last_time = time()
		self.steps = 1
		self.br = tf.TransformBroadcaster()

		self.cmd = cmd

		self.waypoints = waypoints("/home/ekelley/ros_workspace/sandbox/QuadcopterMapping/quadcopterCode/data/waypoints.txt")

		self.current_waypoint = self.waypoints.get_waypoint()

	#Return Euclidean Distance
	def get_distance(self):
		return sqrt((self.current_waypoint.x - self.pose.x)**2 + (self.current_waypoint.y - self.pose.y)**2 + (self.current_waypoint.y - self.pose.y)**2)

	#Get clamped angle difference
	def get_angle_diff(self):
		return abs(clamp_angle(self.current_waypoint.theta - self.pose.theta))

	#Decected marker via ar_pose
	def got_marker(self, data):
		self.localize.ar_correct(data)

	#Update localization
	def update_command(self, data):
		self.last_time = time()
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

		# prnt("Elapsed time: %f" % (time() - self.last_time))
		avg = (time() - self.start_time)/self.steps
		# print("Average time: %f" % (avg))
		print("Command step: %d" % self.steps)
		self.steps += 1
		#Define tilt as being within - and + MAX values

		# if ((time() - self.start_time > 5) and (time() - self.start_time < 10)):
		# 	print("TURNING")
		# 	self.cmd.SetCommand(roll=0,pitch=0,yaw_velocity=.1,z_velocity=0)
		# else:
		# 	self.cmd.SetCommand(roll=0,pitch=0,yaw_velocity=.1,z_velocity=0)
		#SetCommand
		# if (self.start):
		# 	print("Taking off")
		# 	if (self.cmd.status == DroneStatus.Emergency):
		# 		print("Reseting Drone");
		# 		self.cmd.SendEmergency()
		# 	self.cmd.SendTakeoff()
		# 	self.start = False
		# elif (self.cmd.status == DroneStatus.Flying or self.cmd.status == DroneStatus.GotoHover or self.cmd.status == DroneStatus.Hovering):
		# 	print("Flying")
		# 	self.cmd.SetCommand(roll=0,pitch=0,yaw_velocity=0,z_velocity=0)

def main():
	rospy.init_node("controller")
	# cmd = BasicCommands()
	# controller = DroneController(cmd)
	controller = DroneController()
	run = raw_input("Press any key to run:")

	print "Running"
	controller.listener()

if __name__ == '__main__':
	main()


