#!/usr/bin/env python

# Adapted from:
# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('quadcopterCode')
import rospy

from basic_commands import BasicCommands
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui


# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_W
	PitchBackward    = QtCore.Qt.Key.Key_S
	RollLeft         = QtCore.Qt.Key.Key_A
	RollRight        = QtCore.Qt.Key.Key_D
	YawLeft          = QtCore.Qt.Key.Key_Q
	YawRight         = QtCore.Qt.Key.Key_E
	IncreaseAltitude = QtCore.Qt.Key.Key_R
	DecreaseAltitude = QtCore.Qt.Key.Key_F
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	IMU              = QtCore.Qt.Key.Key_I
	Emergency        = QtCore.Qt.Key.Key_Space
	Toggle			 = QtCore.Qt.Key.Key_T


# Extending control_gui to include keypresses
class KeyboardController(DroneVideoDisplay):
	def __init__(self, cmd):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0

		self.linear_gain = .2
		self.angular_gain = .5
		self.cmd = cmd

	def keyPressEvent(self, event):
		key = event.key()
		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if self.cmd is not None and not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.Emergency:
				self.cmd.SendEmergency()
			elif key == KeyMapping.Takeoff:
				self.cmd.SendTakeoff()
			elif key == KeyMapping.Land:
				self.cmd.SendLand()
			elif key == KeyMapping.IMU:
				self.cmd.SendIMUBias()
			elif key == KeyMapping.Toggle:
				self.cmd.SendToggle()
			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += self.angular_gain
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -self.angular_gain

				elif key == KeyMapping.PitchForward:
					self.pitch += self.linear_gain 
				elif key == KeyMapping.PitchBackward:
					self.pitch += -self.linear_gain 

				elif key == KeyMapping.RollLeft:
					self.roll += self.linear_gain 
				elif key == KeyMapping.RollRight:
					self.roll += -self.linear_gain 

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += self.linear_gain 
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -self.linear_gain 

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			self.cmd.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if self.cmd is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= self.angular_gain
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -self.angular_gain

			elif key == KeyMapping.PitchForward:
				self.pitch -= self.linear_gain 
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -self.linear_gain 

			elif key == KeyMapping.RollLeft:
				self.roll -= self.linear_gain 
			elif key == KeyMapping.RollRight:
				self.roll -= -self.linear_gain 

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= self.linear_gain 
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -self.linear_gain 

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			self.cmd.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)



# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_keyboard_controller')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	cmd = BasicCommands()
	display = KeyboardController(cmd)

	display.show()

	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Shutting Down')
	sys.exit(status)