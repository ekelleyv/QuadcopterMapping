#!/usr/bin/env python

# control.py
# Central control unit for controller and localization
# Ed Kelley
# Senior thesis, 2012-2013

import roslib; roslib.load_manifest('quadcopterCode')
import rospy
import sys
from PySide import QtCore, QtGui

import threading

from basic_commands import BasicCommands
from keyboard_controller import KeyboardController
from drone_video_display import DroneVideoDisplay
from drone_controller import DroneController

class Control():
	def __init__(self):
		self.cmd = BasicCommands()
		# self.display = KeyboardController(self.cmd)
		self.drone_controller = DroneController(self.cmd)

	def run(self):
		# self.display.show()
		
		self.drone_controller.listener()
		# executes the QT application
		# self.status = self.app.exec_()

		# and only progresses to here once the application has been shutdown
		rospy.signal_shutdown('Shutting Down')
		self.cmd.SendLand()
		print("AHHHH");
		# sys.exit(self.status)

# Setup the application
if __name__=='__main__':
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_control')
	ctrl = Control()
	ctrl.run()