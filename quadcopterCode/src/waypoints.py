#!/usr/bin/env python

class waypoints:
	def __init__(self, filename):
		self.waypoints = []
		self.new_waypoints = 0
		self.waypoint_index = 0
		read_file(filename)

	def get_waypoint(self):
		last_index = self.waypoints_index
		self.waypoints_index += 1
		return self.waypoints[]

	def add_waypoint(self, x, y, z, theta):
		label = "new" + str(self.new_waypoints)
		self.new_waypoints += 1
		if (self.new_waypoints >= len(self.waypoints)):
			return -1
		else:
			waypoints.append(waypoint(label, x, y, z, theta))

	def read_file(self, filename):
		#Read documentation for reading files
		label = ""
		x = 0
		y = 0
		z = 0
		theta = 0

		waypoints.append(waypoint(label, x, y, z, theta))

	def print_waypoints(self):
		for waypoint in self.waypoints:
			waypoint.print_waypoint()

class waypoint:
	def __init__(self, label, x, y, z, theta):
		self.label = label
		self.x = x
		self.y = y
		self.z = z
		self.theta = theta

	def print_waypoint(self):
		print "%s : (%2f, %2f, %2f, %2f)\n" % (self.x, self.y, self.z, self.theta)

