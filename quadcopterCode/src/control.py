#!/usr/bin/env python

# Adapted from:
# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('quadcopterCode')
import rospy
import sys
from PySide import QtCore, QtGui

from basic_commands import BasicCommands
from keyboard_controller import KeyboardController
from drone_video_display import DroneVideoDisplay
from drone_controller import DroneController

class Control():
	def __init__(self):
		self.app = QtGui.QApplication(sys.argv)
		self.cmd = BasicCommands()
		self.display = KeyboardController(self.cmd)
		self.drone_controller = DroneController(self.cmd)

	def run(self):
		self.display.show()

		# self.drone_controller.listener()
		# executes the QT application
		self.status = self.app.exec_()

		# and only progresses to here once the application has been shutdown
		rospy.signal_shutdown('Shutting Down')
		sys.exit(self.status)

# Setup the application
if __name__=='__main__':
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_control')
	ctrl = Control()
	ctrl.run()