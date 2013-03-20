#!/usr/bin/env python
 
import sys
import gamepad
import time
import math
try:
	#PyGtk library import
 	import pygtk
  	pygtk.require("3.0")
except:
  	pass
try:
	import gtk
  	import gtk.glade
except:
	sys.exit(1)
#ROS related initializations
import roslib; roslib.load_manifest('quadcopterCode')
import rospy
import os
from std_msgs.msg import String, Empty, Float32
from geometry_msgs.msg import Twist


class ControlGUI:

	def __init__(self):
		self.window = gtk.Window()
		self.window.set_title("Quadcopter Control Tower")
		self.window.set_default_size(300, 100)
		self.toggle_mode = True
		self.is_yaw_left = False
		self.is_yaw_right = False
		self.is_alt_up = False
		self.is_alt_down = False
		self.is_pitch_left = False
		self.is_pitch_right = False
		self.is_pitch_forward = False
		self.is_pitch_back = False

		self.yaw_vel = .4
		self.alt_vel = .6
		self.pitch_vel = .1
		# self.window.add_events(gtk.gdk.BUTTON_PRESS_MASK)

		self.create_widgets()
		self.connect_signals()

		self.window.show_all()
		rospy.init_node("controlGUI")

		self.y_est = 0
		self.y_sub = rospy.Subscriber('localize/y_est', Float32, self.update_y)


		

	def create_widgets(self):
		self.vbox = gtk.VBox(spacing=10)
		self.hbox_1 = gtk.HBox(spacing=10)
		self.takeoff_button = gtk.Button("Takeoff")
		self.land_button = gtk.Button("Land")
		self.reset_button = gtk.Button("Reset")
		self.toggle_button = gtk.Button("Toggle Mode")
		self.path_button = gtk.Button("Execute Path")
		self.hbox_1.pack_start(self.takeoff_button)
		self.hbox_1.pack_start(self.land_button)
		self.hbox_1.pack_start(self.reset_button)
		self.hbox_1.pack_start(self.toggle_button)
		self.hbox_1.pack_start(self.path_button)

		self.label = gtk.Label("Welcome to Control Tower")
		self.vbox.pack_start(self.label)
		self.vbox.pack_start(self.hbox_1)

		self.hbox_2 = gtk.HBox(spacing=10)
		self.vbox_left = gtk.VBox(spacing=10)
		self.vbox_right = gtk.VBox(spacing=10)
		self.hbox_left_top = gtk.HBox(spacing=10)
		self.hbox_left_bottom = gtk.HBox(spacing=10)
		self.hbox_right_top = gtk.HBox(spacing=10)
		self.hbox_right_bottom = gtk.HBox(spacing=10)

		self.vbox.pack_start(self.hbox_2)
		self.hbox_2.pack_start(self.vbox_left)
		self.hbox_2.pack_start(self.vbox_right)
		self.vbox_left.pack_start(self.hbox_left_top)
		self.vbox_left.pack_start(self.hbox_left_bottom)
		self.vbox_right.pack_start(self.hbox_right_top)
		self.vbox_right.pack_start(self.hbox_right_bottom)

		self.u1 = gtk.Button("^")
		self.l1 = gtk.Button("<")
		self.r1 = gtk.Button(">")
		self.d1 = gtk.Button("v")

		self.u2 = gtk.Button("^")
		self.l2 = gtk.Button("<")
		self.r2 = gtk.Button(">")
		self.d2 = gtk.Button("v")

		self.hbox_left_top.pack_start(self.u1)
		self.hbox_left_bottom.pack_start(self.l1)
		self.hbox_left_bottom.pack_start(self.d1)
		self.hbox_left_bottom.pack_start(self.r1)

		self.hbox_right_top.pack_start(self.u2)
		self.hbox_right_bottom.pack_start(self.l2)
		self.hbox_right_bottom.pack_start(self.d2)
		self.hbox_right_bottom.pack_start(self.r2)

		self.window.add(self.vbox)

	def connect_signals(self):
		self.takeoff_button.connect("clicked", self.takeoff)
		self.land_button.connect("clicked", self.land)
		self.reset_button.connect("clicked", self.reset)
		self.toggle_button.connect("clicked", self.toggle)
		self.path_button.connect("clicked", self.execute_path)

		self.u1.connect("pressed", self.pitch_forward)
		self.l1.connect("pressed", self.pitch_left)
		self.d1.connect("pressed", self.pitch_back)
		self.r1.connect("pressed", self.pitch_right)

		self.u2.connect("pressed", self.alt_up)
		self.l2.connect("pressed", self.yaw_left)
		self.d2.connect("pressed", self.alt_down)
		self.r2.connect("pressed", self.yaw_right)

		# self.window.connect("key-press-event", self.keypress)

		self.window.connect("destroy", self.destroy_window)

	# def keypress(self, widget, event):
	# 	# print "%d" % event.get_state()
	# 	return

	def takeoff(self, widget):
		#Simple button cliked event
		self.hover()
		print "Taking off!"
		self.takeoff_pub.publish(Empty())


	def land(self, widget):
		#Simple button cliked event
		
		print "Landing!"
		self.land_pub.publish(Empty())

	def reset(self, widget):
		#Simple button cliked event
		
		print "Reset!"
		self.reset_pub.publish(Empty())
		self.hover()

	def toggle(self, widget):
		self.toggle_mode = not self.toggle_mode

		if (self.toggle_mode):
			print "Toggle mode engaged"
		else:
			print "Out of toggle mode"

	def yaw_left(self, widget):
		print("Yaw left")
		self.is_yaw_left = not self.is_yaw_left

		if (self.toggle_mode):
			if (self.is_yaw_left):
				self.hover()
				self.twist.angular.z=self.yaw_vel
				self.twist_pub.publish(self.twist)
			else:
				self.hover()
		else:
			self.hover()
			self.twist.angular.z=self.yaw_vel
			self.twist_pub.publish(self.twist)
			time.sleep(.5)
			self.hover()

	def yaw_right(self, widget):
		print("Yaw right")
		if (self.toggle_mode):
			self.is_yaw_right = not self.is_yaw_right
			if (self.is_yaw_right):
				self.hover()
				self.twist.angular.z=-self.yaw_vel
				self.twist_pub.publish(self.twist)
			else:
				self.hover()
		else:
			self.hover()
			self.twist.angular.z=-self.yaw_vel
			self.twist_pub.publish(self.twist)
			time.sleep(.5)
			self.hover()

	def alt_up(self, widget):
		print("Up")

		if (self.toggle_mode):
			self.is_alt_up = not self.is_alt_up
			if (self.is_alt_up):
				self.hover()
				self.twist.linear.z=self.alt_vel
				self.twist_pub.publish(self.twist)
			else:
				self.hover()
		else:
			self.hover()
			self.twist.linear.z=self.alt_vel
			self.twist_pub.publish(self.twist)
			time.sleep(.5)
			self.hover()

	def alt_down(self, widget):
		print("Down")
		if (self.toggle_mode):
			self.is_alt_down = not self.is_alt_down
			if (self.is_alt_down):
				self.hover()
				self.twist.linear.z=-self.alt_vel
				self.twist_pub.publish(self.twist)
			else:
				self.hover()
		else:
			self.hover()
			self.twist.linear.z=-self.alt_vel
			self.twist_pub.publish(self.twist)
			time.sleep(.5)
			self.hover()

	def pitch_left(self, widget):
		print("Pitch left")
		if (self.toggle_mode):
			self.is_pitch_left = not self.is_pitch_left
			if (self.is_pitch_left):
				self.hover()
				self.twist.linear.y=self.pitch_vel
				self.twist_pub.publish(self.twist)
			else:
				self.hover()
		else:
			self.hover()
			self.twist.linear.y=self.pitch_vel
			self.twist_pub.publish(self.twist)
			time.sleep(.5)
			self.hover()

	def pitch_right(self, widget):
		print("Pitch right")
		if (self.toggle_mode):
			self.is_pitch_right = not self.is_pitch_right
			if (self.is_pitch_right):
				self.hover()
				self.twist.linear.y=-self.pitch_vel
				self.twist_pub.publish(self.twist)
			else:
				self.hover()
		else:
			self.hover()
			self.twist.linear.y=-self.pitch_vel
			self.twist_pub.publish(self.twist)
			time.sleep(.5)
			self.hover()

	def pitch_forward(self, widget):
		print("Pitch forward")
		if (self.toggle_mode):
			self.is_pitch_forward = not self.is_pitch_forward
			if (self.is_pitch_forward):
				self.hover()
				self.twist.linear.x=self.pitch_vel
				self.twist_pub.publish(self.twist)
			else:
				self.hover()
		else:
			self.hover()
			self.twist.linear.x=self.pitch_vel
			self.twist_pub.publish(self.twist)
			time.sleep(1)
			self.hover()

	def pitch_back(self, widget):
		print("Pitch back")
		if (self.toggle_mode):
			self.is_pitch_back = not self.is_pitch_back
			if (self.is_pitch_back):
				self.hover()
				self.twist.linear.x=-self.pitch_vel
				self.twist_pub.publish(self.twist)
			else:
				self.hover()
		else:
			self.hover()
			self.twist.linear.x=-self.pitch_vel
			self.twist_pub.publish(self.twist)
			time.sleep(1)
			self.hover()

	def hover(self):
		self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
		self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
		self.twist_pub.publish(self.twist)
	def getKey():
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		print "You pressed " + key
		return key
	def destroy_window(self,widget):
		#MainWindow_destroy event
		sys.exit(0)

	def update_y(self, data):
		self.y_est = data
		print "Y is %f" % self.y_est 

	def execute_path(self, widget):
		self.takeoff(widget)
		time.sleep(5)
		# self.twist.linear.y = -.1
		# self.twist_pub.publish(self.twist)
		y_des = -2000.0
		distance = math.fabs(self.y_est-y_des)
		while(distance > 1000):
			print("Distance is %f" % distance)
			time.sleep(.001)
		self.hover()
		time.sleep(5)
		self.land(widget)

if __name__ == "__main__":
	#start the class
	cg = controlGUI()
	gtk.main()#Starts GTK