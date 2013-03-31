#!/usr/bin/env python
import os, sys

class waypoints:
	def __init__(self, filename):
		self.waypoint_list = []
		self.new_waypoints = 0
		self.waypoint_index= 0
		self.read_file(filename)

	def get_waypoint(self):
		last_index = self.waypoint_index
		self.waypoint_index += 1
		return self.waypoint_list[self.waypoint_index-1]

	def add_waypoint(self, x, y, z, theta):
		label = "new" + str(self.new_waypoints)
		self.new_waypoints += 1
		self.waypoint_list.append(waypoint(label, x, y, z, theta))

	def read_file(self, filename):
		#Read documentation for reading files
		f = open(filename)

		for line in f:
			vals = line.split()
			label = vals[0]
			x = int(vals[1])
			y = int(vals[2])
			z = int(vals[3])
			theta = int(vals[4])

			self.waypoint_list.append(waypoint(label, x, y, z, theta))

	def print_waypoints(self):
		for waypoint in self.waypoint_list:
			waypoint.print_waypoint()

class waypoint:
	def __init__(self, label, x, y, z, theta):
		self.label = label
		self.x = x
		self.y = y
		self.z = z
		self.theta = theta

	def print_waypoint(self):
		print "%s : (%f, %f, %f, %f)" % (self.label, self.x, self.y, self.z, self.theta)


def main():
	#Tests
	test = waypoints("/home/ekelley/ros_workspace/sandbox/QuadcopterMapping/quadcopterCode/data/waypoints.txt")
	print "File Waypoints"
	test.print_waypoints()

	test.add_waypoint(5, 5, 5, 5)
	print "Add Waypoints"
	test.print_waypoints()

	print "Getting 0"
	waypoint = test.get_waypoint()
	waypoint.print_waypoint()

	print "Getting 1"
	waypoint = test.get_waypoint()
	waypoint.print_waypoint()

if __name__ == '__main__':
	main()