#ifndef _MAIN_CPP_
#define _MAIN_CPP_
/*
 main.cpp
 Main control loop
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */

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

using namespace std;

/* 
 * ROS INCLUDES
 */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
 #include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Twist.h>

/*
 * ARDRONE_AUTONOMY INCLUDES
 */
#include "ardrone_autonomy/Navdata.h"

/*
 * TUM_ARDRONE INCLUDES
 */
#include "tum_ardrone/filter_state.h"

/*
 * PROJECT INCLUDES
 */
#include "Drone.h"


/*
 * DEFINITIONS
 */
#define MY_MASK 0777 //for setting folder permissions when using mkdir
//path to log files, make sure trailing / is there
#define ROS_WORKSPACE "/home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterControl/bin/" 

/*
 * GLOBAL VARIABLES
 */
std::fstream navLog; //name of log file
std::fstream predictLog; //log predicted things
long programStart; //time of program start
struct tm *now; //beginning of programming

/*
 FUNCTION DECLARATIONS
 */
long myclock();
void navDataCB(const ardrone_autonomy::Navdata::ConstPtr& msg);
void predictDataCB(const tum_ardrone::filter_state::ConstPtr& msg);
void imgCB(const sensor_msgs::ImageConstPtr& msg);

/*
 * FUNCTIONS
 */

/* 
 * Returns a time value.
 */
long myclock() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000000) + tv.tv_usec;
}

/* 
 * Callback function when navdata is updated. Logs navData in logfile. 
 */
void navDataCB(const ardrone_autonomy::Navdata::ConstPtr& msg) {
	//get elapsed time
   	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	//log all variables
	navLog << "time= " << currentTime << " droneTime= " << msg->tm << " batteryPrecent= " << msg->batteryPercent << " state= " << msg->state << " rot= " << msg->rotX << " " << msg->rotY << " " << msg->rotZ << " altd= " << msg->altd << " linearV= " << msg->vx << " " << msg->vy << " " << msg->vz << " " << " linearAccel " << msg->ax << " " << msg->ay << " " << msg->az << "\n";
}

/* 
 * Callback function when predictdata is updated. Logs predictData in logfile. 
 */
void predictDataCB(const tum_ardrone::filter_state::ConstPtr& msg) {
	//get elapsed time
   	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	//log all variables
	predictLog << "time= " << currentTime << " messageTime= " << msg->header.stamp << " batteryPrecent= " << msg->batteryPercent << " state= " << msg->droneState << " pos = " << msg->x << " " << msg-> y << " " << msg->z << " linearV= " << msg->dx << " " << msg->dy << " " << msg->dz << " rot= " << msg->pitch << " " << msg->roll << " " << msg->yaw << " dyaw= " << msg->dyaw << " scale= " << msg->scale << " ptamState= " << msg->ptamState << " scaleAccuracy= " << msg->scaleAccuracy << "\n";
}

/* 
 * Callback function when new image is recieved in front-facing camera. Stored as .jpgs in folder.  
 */
void imgCB(const sensor_msgs::ImageConstPtr& msg) {
	//convert image from ros image to open CV image
	sensor_msgs::CvBridge bridge;
	IplImage* img;
  	try {
		img = bridge.imgMsgToCv(msg, "bgr8");
  	}
  	catch (sensor_msgs::CvBridgeException& e) {
    		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	} 

	//display image
	cvShowImage("view", img);

	//get elapsed time
	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	//save .jpg with timestamped name in ROS_WORKSPACE/[programStart]
	std::stringstream time;
	time << ROS_WORKSPACE << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << "/" << currentTime << ".jpg";
	const std::string tmp = time.str();
	const char* timeStamp = tmp.c_str();

	cvSaveImage(timeStamp, img);
}

/*
 *MAIN
 */
int main(int argc, char** argv) {
	//INITIALIZE LOGGING
	//get current time
   	programStart = myclock();
   	time_t t = time(0);
   	now = localtime(&t);

	//make directory for images, named ROS_WORKSPACE/[datetime]
	std::stringstream imgDir;
	imgDir << ROS_WORKSPACE << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min;
	const std::string tmp = imgDir.str();
	const char* imgDirectory = tmp.c_str();

  	int temp;
	temp = umask(0);
  	if ((temp = mkdir(imgDirectory, MY_MASK)) != 0) {
    		fprintf(stderr, "ERROR %d: unable to mkdir; %s\n", errno, strerror(errno));
  	}

	//make log file, named ROS_WORKSPACE/navLog_[datetime]
	std::stringstream navLogStr;
	navLogStr << ROS_WORKSPACE << "navLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	const std::string tmp2 = navLogStr.str();
	const char* navLogName = tmp2.c_str();
	navLog.open(navLogName, std::ios_base::out);

	std::stringstream predictLogStr;
	predictLogStr << ROS_WORKSPACE << "predictLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	const std::string tmp2_pr = predictLogStr.str();
	const char* predictLogName = tmp2_pr.c_str();
	predictLog.open(predictLogName, std::ios_base::out);

	std_msgs::Empty empty;
	std_srvs::Empty emptyCall;

	//INITALIZE ROS NODES
	ros::init(argc, argv, "test_ardrone_autonomy");
	ros::NodeHandle n;

	//subscribe to camera feeds
	image_transport::ImageTransport it(n);
	image_transport::Subscriber frontCam_sub = it.subscribe("/ardrone/front/image_raw", 1, imgCB); //forward camera
	//image_transport::Subscriber bottomCam_sub = it.subscribe("/ardrone/bottom/image_raw", 1, imgCB); //downward camera

	//service call to toggle camera
	ros::ServiceClient toggleCam = n.serviceClient<std_srvs::Empty>("/ardrone/togglecam");

	//subscribe to navdata
	ros::Subscriber navdata_sub = n.subscribe("/ardrone/navdata", 1000, navDataCB);
	ros::Subscriber predictdata_sub = n.subscribe("/ardrone/predictedPose", 1000, predictDataCB);

	//make openCV windows
	cvNamedWindow("view");
  	cvStartWindowThread();

	//initialize cmd_vel
	ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("\cmd_vel", 1000);
	while (cmd_vel.getNumSubscribers() < 1) { ;}
	geometry_msgs::Twist velocity;
	velocity.linear.x = 0;
	velocity.linear.y = 0;
	velocity.linear.z = 0;	
	velocity.angular.x = 0;
	velocity.angular.y = 0;
	velocity.angular.z = 0;
	cmd_vel.publish(velocity);
	ros::spinOnce(); //reset velocity

	//command take off
	ROS_INFO("Taking off..");	
	ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
	while (takeoff.getNumSubscribers() < 1) { ;}
	//takeoff.publish(empty);
	//ros::spinOnce();

	int32_t controlFlag = 0;

	//hover
	bool cam = false;
	bool ind = false;
	while (ros::ok()) {
		//process ros messages
		ros::spinOnce();

		//get elapsed time
   		long end = myclock();
        	long currentTime = (end-programStart)/1000000.0;

		if (currentTime > 30) { break;}
		if ((currentTime > 4) && (ind == false)) {
			ROS_INFO("Sending command to turn...");
			geometry_msgs::Twist velocity;
			velocity.linear.x = -0.5;
			velocity.linear.y = 0.0;
			velocity.linear.z = 0.0;	
			velocity.angular.x = 0.0;
			velocity.angular.y = 0.0;
			velocity.angular.z = 0.0;

			cmd_vel.publish(velocity);
			ros::spinOnce();

			ind = true;
			
	  		//ardrone_at_set_progress_cmd(controlFlag, 0.0, 0.0, 0.0, 0.2);
			//ardrone_tool_set_progressive_cmd(controlFlag, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		}
		//command to toggle camera
		/*else if (currentTime > 5 && cam == false) {
			if (toggleCam.call(emptyCall)) { ROS_INFO("Toggling camera");}
			else { ROS_ERROR("Failed to call toggle camera service");}

			cam = true;
		}*/
	}

	//land the quad
//	ROS_INFO("Sending command to stop turning...");
	//ardrone_tool_set_progressive_cmd(controlFlag, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//	  ardrone_at_set_progress_cmd(input_state.pcmd.flag, input_state.pcmd.phi, input_state.pcmd.theta, input_state.pcmd.gaz, input_state.pcmd.yaw);

	ROS_INFO("Landing..");
	ros::Publisher land = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
	while (land.getNumSubscribers() < 1) { ;}
	land.publish(empty);
	ros::spinOnce();
	
	//exit cleanly
	cvDestroyWindow("view");
	navLog.close();

	return 0;

}

#endif //_CUSTOMUSER_CPP_
