#!/usr/bin/env python

# utils.py
# Ed Kelley
# Senior thesis, 2012-2013

import numpy
from math import *

#Rotate to global coordinates
def rotate(m, rotX, rotY, rotZ):
	rotX_m = numpy.matrix([[ 1, 0, 0, 0],
							[ 0, cos(radians(rotX)), -sin(radians(rotX)), 0],
							[ 0, sin(radians(rotX)), cos(radians(rotX)), 0],
							[ 0, 0, 0, 1]])
	rotY_m = numpy.matrix([[cos(radians(rotY)), 0, -sin(radians(rotY)), 0],
							[ 0, 1, 0, 0],
							[ sin(radians(rotY)), 0, cos(radians(rotY)), 0],
							[ 0, 0, 0, 1]])
	rotZ_m = numpy.matrix([[cos(radians(rotZ)), -sin(radians(rotZ)), 0, 0],
							[sin(radians(rotZ)),cos(radians(rotZ)),0,0],
							[0,0,1,0],
							[0,0,0,1]])
	return rotZ_m*rotY_m*rotX_m*m;

#Keep angle between -180 and 180
def clamp_angle(angle):
	if (angle > 180):
		return angle - 360
	elif (angle < -180):
		return angle + 360
	else:
		return angle

#Adapted from:
#http://stackoverflow.com/questions/12412895/calculate-probability-in-normal-distribution-given-mean-std-in-python
def normpdf(x, mean, sd):
    var = float(sd)**2
    denom = (2*pi*var)**.5
    num = exp(-(float(x)-float(mean))**2/(2*var))
    return num/denom

# Adapted from:
# http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
def get_heading(magX, magY, magZ):
	
	heading = (atan2(magX, magY)/pi)*180
	return heading

def convert_g(acc):
	g_to_mmss = 9806.65
	return acc*g_to_mmss



