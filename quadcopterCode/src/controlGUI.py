#!/usr/bin/env python
 
import sys
import gamepad
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
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
class controlGUI:
	def __init__(self):
		#Set the path to Glade file, in the ros_glade ROS package
		# str=roslib.packages.get_pkg_dir('quadcopterCode')+"/src/gui/quadcopterGUITest.glade"
		# self.gladefile = str 
		# #Initiate the Builder and point it to the glade file
		# self.builder = gtk.Builder()
		# self.builder.add_from_file(self.gladefile)
		# #Connect event functions
		# self.builder.connect_signals(self)
		self.window = gtk.Window()
		self.window.set_title("Quadcopter Control Tower")
		self.window.set_default_size(300, 100)

		self.create_widgets()
		self.connect_signals()

		self.window.show_all()
		self.takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty)
		self.land_pub = rospy.Publisher('ardrone/land', Empty)
		self.reset_pub = rospy.Publisher('ardrone/reset', Empty)
		self.twist_pub = rospy.Publisher('cmd_vel', Twist)
		rospy.init_node('talker')
		self.twist = Twist()

		

	def create_widgets(self):
		self.vbox = gtk.VBox(spacing=10)
		self.hbox_1 = gtk.HBox(spacing=10)
		self.takeoff_button = gtk.Button("Takeoff")
		self.land_button = gtk.Button("Land")
		self.reset_button = gtk.Button("Reset")
		self.hbox_1.pack_start(self.takeoff_button)
		self.hbox_1.pack_start(self.land_button)
		self.hbox_1.pack_start(self.reset_button)

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

		self.u1.connect("pressed", self.pitch_forward)
		self.l1.connect("pressed", self.pitch_left)
		self.d1.connect("pressed", self.pitch_back)
		self.r1.connect("pressed", self.pitch_right)

		self.u2.connect("pressed", self.alt_up)
		self.l2.connect("pressed", self.yaw_left)
		self.d2.connect("pressed", self.alt_down)
		self.r2.connect("pressed", self.yaw_right)

		self.window.connect("destroy", self.destroy_window)

	def takeoff(self, widget):
		#Simple button cliked event
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

	def talker(self):
	    	#ROS message hello world
	    if not rospy.is_shutdown():
		str = "hello world %s"%rospy.get_time()
		rospy.loginfo(str)
		self.pub.publish(String(str))

	def yaw_left(self, widget):
		print("Yaw left")
		self.hover()
		self.twist.angular.z=-1
		self.twist_pub.publish(self.twist)	
		time.sleep(1)	
		self.twist.angular.z=0
		self.twist_pub.publish(self.twist)


	def yaw_right(self, widget):
		print("Yaw right")

	def alt_up(self, widget):
		print("Up")

	def alt_down(self, widget):
		print("Down")

	def pitch_left(self, widget):
		print("Pitch left")

	def pitch_right(self, widget):
		print("Pitch right")

	def pitch_forward(self, widget):
		print("Pitch forward")

	def pitch_back(self, widget):
		print("Pitch back")

	def hover(self):
		self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
		self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
		self.twist_pub.publish(self.twist)

	def destroy_window(self,widget):
		#MainWindow_destroy event
		sys.exit(0)
if __name__ == "__main__":
	#start the class
	cg = controlGUI()
	gtk.main()#Starts GTK