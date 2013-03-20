QuadcopterMapping
=================

Mapping large objects using multiple autonomous ARDrones.
-----------------

Notes:
	-To kill ghost ros nodes and processes: 
		-rosnode list: to get a list of all running nodes
		-rosnode kill [nameOfNode]
		-rosnode cleanup: if kill didn't kill it

Architecture:
	-quadcopterVision
		-main.cpp
			-Subscribes to image stream and tracks points
	-quadcopterLocalization
	-quadcopterCode


Run driver:
	-rosrun ardrone_autonomy ardrone_driver


Webcam info:
	roscd gscam
	cd bin
	export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
	rosrun gscam gscam

	Publishes to gscam/image_raw
	Search for device in /dev/video* (should be 0)

Quadcopter Image Stream:
	/ardrone/image_raw
	Rectify images by using:
		ROS_NAMESPACE=ardrone/front rosrun image_proc image_proc
	Check rectified:
		rosrun image_view image_view image:=ardrone/front/image_rect_color

Running Tag Identification:
	roslaunch ar_track_alvar ardrone_indiv_no_kinect.launch

Resolving issues with Eigen
	Added macros to Markers.h in ar_track_alvar

Log
==================

-2/17/13
	-- Figured out rectifying images
	-- Researched using tf
	-- Looked at kalman filter package
-2/18/13
	-- Debugging ar_track_alvar issues
	-- Asked Tower for webcam