#!/usr/bin/env python
 
import sys
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
		self.window.set_default_size(300, 300)

		self.create_widgets()
		self.connect_signals()

		self.window.show_all()
		self.takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty)
		self.land_pub = rospy.Publisher('ardrone/land', Empty)
		self.reset_pub = rospy.Publisher('ardrone/reset', Empty)
		rospy.init_node('talker')

		

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

		self.window.add(self.vbox)

	def connect_signals(self):
		self.takeoff_button.connect("clicked", self.takeoff)
		self.land_button.connect("clicked", self.land)
		self.reset_button.connect("clicked", self.reset)

		self.window.connect("destroy", self.destroy_window)

	def takeoff(self, widget):
		#Simple button cliked event
		print "Taking off!"
		self.takeoff_pub.publish()

	def land(self, widget):
		#Simple button cliked event
		
		print "Landing!"
		self.land_pub.publish()

	def reset(self, widget):
		#Simple button cliked event
		
		print "Reset!"
		self.reset_pub.publish()

	def talker(self):
	    	#ROS message hello world
	    if not rospy.is_shutdown():
		str = "hello world %s"%rospy.get_time()
		rospy.loginfo(str)
		self.pub.publish(String(str))

	def destroy_window(self,widget):
		#MainWindow_destroy event
		sys.exit(0)
if __name__ == "__main__":
	#start the class
	cg = controlGUI()
	# gtk.timeout_add(1000, cg.Timer1_timeout) #Adds a timer to the GUI, with hwg.Timer1_timeout as a 
	#callback function for the timer1
	gtk.main()#Starts GTK