#ifndef _MAIN_CPP_
#define _MAIN_CPP_
/*
 main.cpp
 Adapted from: http://siddhantahuja.wordpress.com/2011/07/20/working-with-ros-and-opencv-draft/
 Handling of online vision tasks
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */

//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/nonfree.hpp>

/*
 * C++ INCLUDES
 */
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

 #define ROS_WORKSPACE "/home/ekelley/"

using namespace std;
using namespace cv;


//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

long programStart; //time of program start
struct tm *now; //beginning of programming
int image_number;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

long myclock() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000000) + tv.tv_usec;
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	Mat mat;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
		mat = cv_ptr->image;
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("quadcopterVision::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}

	long end = myclock();
    long currentTime = (end-programStart)/1000000.0;

	// //Save image
	// std::stringstream folder;
	// folder << ROS_WORKSPACE 
	// 	<< now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min;
	// const std::string tmp = folder.str();
	// const char* foldertimeStamp = tmp.c_str();

    // time_t rawtime;
    // struct tm* timeinfo;
    // char folder_buffer [80];
    // char file_buffer [80];

    // time(&rawtime);
    // time info = localtime(&rawtime);
    // strftime (folder_buffer, 80, "%Y-%m-%d-%H-%M", timeinfo);
    // strftime (file_buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
	system("mkdir -p /home/ekelley/images");
	std::stringstream image_string;
	image_string << image_number;
	imwrite("/home/ekelley/images/" + image_string.str() + ".jpg", mat);
	image_number++;
	
	cv::SiftFeatureDetector a;
	vector<KeyPoint> keypoints;
	a.detect(mat, keypoints);

	drawKeypoints(mat, keypoints, mat);
	//Display the image using OpenCV
	cv::imshow(WINDOW, mat);
	//Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
	cv::waitKey(3);
	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor in main().
	*/
	//Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
}

/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
	* The name used here must be a base name, ie. it cannot have a / in it.
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ROS_INFO("quadcopterVision::main.cpp::STARTING.");
    ros::init(argc, argv, "image_processor");

    programStart = myclock();
   	time_t t = time(0);
   	now = localtime(&t);
   	image_number = 0;
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
        ros::NodeHandle nh;
	//Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
	//OpenCV HighGUI call to create a display window on start-up.
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	/**
	* Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used. 
	* In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call 
	* the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
	* subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe. 
	* When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
	*/
    image_transport::Subscriber sub = it.subscribe("ardrone/image_raw", 1, imageCallback);
	//OpenCV HighGUI call to destroy a display window on shut-down.
	cv::destroyWindow(WINDOW);
	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
        pub = it.advertise("camera/image_processed", 1);
	/**
	* In this application all user callbacks will be called from within the ros::spin() call. 
	* ros::spin() will not return until the node has been shutdown, either through a call 
	* to ros::shutdown() or a Ctrl-C.
	*/
        ros::spin();
	//ROS_INFO is the replacement for printf/cout.
	ROS_INFO("quadcopterVision::main.cpp::No error.");

}

#endif //_CUSTOMUSER_CPP_