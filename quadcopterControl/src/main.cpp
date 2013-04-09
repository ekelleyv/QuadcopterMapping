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
#include "functions.h"


/*
 * DEFINITIONS
 */
#define MY_MASK 0777 //for setting folder permissions when using mkdir
//path to log files, make sure trailing / is there
#define ROS_WORKSPACE "/home/sytang/Dropbox/ros_workspace/QuadcopterMapping/quadcopterControl/bin/" 

/*
 * GLOBAL VARIABLES
 */
std::fstream settingsLog; //settings of the trial
std::fstream navLog; //name of log file
std::fstream predictLog; //log predicted things
std::fstream controlLog; //log control commands
int updateType = 2; //0 for navdata, 1 for ptam, 2 for integrate with ptam, 3 for integrate with navdata
long programStart; //time of program start
struct tm *now; //beginning of programming
Drone* droneP;

/*
 FUNCTION DECLARATIONS
 */
long myclock();
void navDataCB(const ardrone_autonomy::Navdata::ConstPtr& msg);
void predictDataCB(const tum_ardrone::filter_state::ConstPtr& msg);
void imgCB(const sensor_msgs::ImageConstPtr& msg);
void cmdVelCB(const geometry_msgs::TwistConstPtr& msg);

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
	navLog << "time= " << currentTime << " messageTime= " << msg->header.stamp << " droneTime= " << msg->tm << " lastStateTime= " << droneP->lastStateTime << " batteryPrecent= " << msg->batteryPercent << " state= " << msg->state << " originalPosition= " << droneP->_x << " " << droneP->_y << " rot= " << msg->rotX << " " << msg->rotY << " " << msg->rotZ << " altd= " << msg->altd << " linearV= " << msg->vx << " " << msg->vy << " " << msg->vz << " " << " linearAccel= " << msg->ax << " " << msg->ay << " " << msg->az << " mag= " << msg->magX << " " << msg->magY << " " << msg->magZ << " pressure= " << msg->pressure << " temp= " << msg->temp << " windSpeed= " << msg->wind_speed << " windAngle= " << msg->wind_angle << " windCompAngle= " << msg->wind_comp_angle;

	droneP->updateNavState(msg);
	droneP->updateState();

	navLog << " newPosition= " << droneP->_x << " " << droneP->_y << " angles= " << droneP->_theta << " " << droneP->_phi << " " << droneP->_psi << " altitude= " << droneP->_alt << " velocities= " << droneP->_u << " " << droneP->_v << " " << droneP->_w << " accelerations= " << droneP->_udot << " " << droneP->_vdot << " " << droneP->_wdot << "\n";
}

/* 
 * Callback function when predictdata is updated. Logs predictData in logfile. 
 */
void predictDataCB(const tum_ardrone::filter_state::ConstPtr& msg) {
	//get elapsed time
   	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	droneP->updateTumState(msg);
	droneP->updateState();

	//log all variables
	predictLog << "time= " << currentTime << " messageTime= " << msg->header.stamp.toSec() << " batteryPrecent= " << msg->batteryPercent << " state= " << msg->droneState << " pos= " << msg->x << " " << msg-> y << " " << msg->z << " linearV= " << msg->dx << " " << msg->dy << " " << msg->dz << " rot= " << msg->pitch << " " << msg->roll << " " << msg->yaw << " dyaw= " << msg->dyaw << " scale= " << msg->scale << " ptamState= " << msg->ptamState << " scaleAccuracy= " << msg->scaleAccuracy;

	predictLog << " newPosition= " << droneP->_x << " " << droneP->_y << " angles= " << droneP->_theta << " " << droneP->_phi << " " << droneP->_psi << " altitude= " << droneP->_alt << " velocities= " << droneP->_u << " " << droneP->_v << " " << droneP->_w << " accelerations= " << droneP->_udot << " " << droneP->_vdot << " " << droneP->_wdot << "\n";
}

void cmdVelCB(const geometry_msgs::TwistConstPtr& msg) {
	//get elapsed time
   	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	//log control 
	controlLog << "time= " << currentTime << " messageTime = " << ros::Time::now() << " command= " << msg->linear.x << " " << msg->linear.y << " " << msg->linear.z << " " << msg->angular.x << " " << msg->angular.y << " " << msg->angular.z << "\n";
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

	//cvSaveImage(timeStamp, img);
}

/*
 *MAIN
 */
int main(int argc, char** argv) {
	/***INITALIZE ROS NODES***/
	ROS_INFO("Initializing nodes...");
	ros::init(argc, argv, "ardrone_control");
	ros::NodeHandle n;

	//maximums for control signal - read from command line, if none specified, use default
	double maxAngle = -1;
	double maxPsiDot = -1;
	double maxZDot = -1;
	processCmdLine(argc, argv, &maxAngle, &maxZDot, &maxPsiDot);

std::cout << "Maximums: " << maxAngle << " " << maxPsiDot << " " << maxZDot << std::endl;

	/***INITIALIZE LOGGING***/
	ROS_INFO("Initializing logs...");
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

	//make log files, named ROS_WORKSPACE/[type]Log_[datetime]
	std::stringstream settingsStr;
	settingsStr << ROS_WORKSPACE << "settings" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	const std::string tmps = settingsStr.str();
	const char* settingsName = tmps.c_str();
	settingsLog.open(settingsName, std::ios_base::out);

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

	std::stringstream controlLogStr;
	controlLogStr << ROS_WORKSPACE << "controlLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	const std::string tmp3_pr = controlLogStr.str();
	const char* controlLogName = tmp3_pr.c_str();
	controlLog.open(controlLogName, std::ios_base::out);

	std_msgs::Empty empty;
	std_srvs::Empty emptyCall;

	/***SET UP SERVICES***/
	//subscribe to camera feeds
	image_transport::ImageTransport it(n);
	image_transport::Subscriber frontCam_sub = it.subscribe("/ardrone/front/image_raw", 1, imgCB); //forward camera
	//image_transport::Subscriber bottomCam_sub = it.subscribe("/ardrone/bottom/image_raw", 1, imgCB); //downward camera

	//service call to toggle camera
	ros::ServiceClient toggleCam = n.serviceClient<std_srvs::Empty>("/ardrone/togglecam");

	//subscribe to navdata
	ros::Subscriber navdata_sub = n.subscribe("/ardrone/navdata", 1000, navDataCB);
	ros::Subscriber predictdata_sub = n.subscribe("/ardrone/predictedPose", 1000, predictDataCB);

	ROS_INFO("Initializing openCV...");
	//make openCV windows
	cvNamedWindow("view");
  	cvStartWindowThread();

	ROS_INFO("Initialize cmd_vel..");
	//initialize cmd_vel
	ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
ROS_INFO("OK1");
	//DON'T FORGET TO PUT THIS CHECK BACK
	while (cmd_vel.getNumSubscribers() < 1) { ;}
ROS_INFO("OK2");
	geometry_msgs::Twist velocity;
ROS_INFO("OK3");
	velocity.linear.x = 0;
	velocity.linear.y = 0;
	velocity.linear.z = 0;	
	velocity.angular.x = 0;
	velocity.angular.y = 0;
	velocity.angular.z = 0;
ROS_INFO("OK4");
	//cmd_vel.publish(velocity);
ROS_INFO("OK5");
	//ros::spinOnce(); //reset velocity
	//ros::Subscriber cmdvel_sub = n.subscribe("/cmd_vel", 1000, cmdVelCB);
ROS_INFO("OK6");
	/***INITIALIZE DRONE STUFF***/
	ROS_INFO("Initialize drone stuff...");
	//initialize desired waypoints
	int numPoints = 6;
	int currentWaypoint = 0;
	//assume units of mm
	double* xDes = new double[numPoints];
	double* yDes = new double[numPoints];
	double* aDes = new double[numPoints];
	double* tDes = new double[numPoints];	

	for (int i = 0; i < numPoints; i++) {
		xDes[i] = 0;
		yDes[i] = 0;
		tDes[i] = -180;
		aDes[i] = 700;
	}

	/*xDes[0] = 500;
	xDes[1] = 1000;
	xDes[2] = 1200;
	xDes[3] = 1700;*/

	/*yDes[0] = -600;
	yDes[1] = -200;
	yDes[2] = 400;
	yDes[3] = 800;*/

	//square
	/*xDes[0] = 0;
	xDes[1] = 1000;
	xDes[2] = 1000;
	xDes[3] = 0;
	xDes[4] = 0;

	yDes[0] = 0;
	yDes[1] = 0;
	yDes[2] = -600;
	yDes[3] = -600;
	yDes[4] = 0;*/

	//sine wave
	/*xDes[0] = 50;
	xDes[1] = 450;
	xDes[2] = 850;
	xDes[3] = 1250;
	xDes[4] = 1650;
	xDes[5] = 2050;
	xDes[6] = 2450;

	yDes[0] = 400;
	yDes[1] = 800;
	yDes[2] = 400;
	yDes[3] = -400;
	yDes[4] = -800;
	yDes[5] = -400;
	yDes[6] = 0;*/

	//sine2
	xDes[0] = 50;
	xDes[1] = 300;
	xDes[2] = 500;
	xDes[3] = 800;
	xDes[4] = 1100;
	xDes[5] = 1400;
	xDes[6] = 1700;

	aDes[0] = 1200;
	aDes[1] = 1500;
	aDes[2] = 1200;
	aDes[3] = 1000;
	aDes[4] = 800;
	aDes[5] = 1000;
	aDes[6] = 1200;

	//star
	/*yDes[0] = 600;
	yDes[1] = -600;
	yDes[2] = 100;
	yDes[3] = 0;
	yDes[4] = -100;
	yDes[5] = 600;

	xDes[0] = 0;
	xDes[1] = 0;
	xDes[2] = -400;
	xDes[3] = 400;
	xDes[4] = -400;
	xDes[5] = 0;*/


	//log waypoints
	settingsLog << " NumberWaypoints= " << numPoints << " xDes= ";
	for (int i = 0; i < numPoints; i++) {
		settingsLog << xDes[i] << " ";
	} 
	settingsLog << "yDes= ";
	for (int i = 0; i < numPoints; i++) {
		settingsLog << yDes[i] << " ";
	}
	settingsLog << " altDes= ";
	for (int i = 0; i < numPoints; i++) {
		settingsLog << aDes[i] << " ";
	}
	settingsLog << " psiDes= ";
	for (int i = 0; i < numPoints; i++) {
		settingsLog << tDes[i] << " ";
	}
	settingsLog << "\n";

/*for (int i = 0; i < numPoints; i++) {
		std::cout << "(" << xDes[i] << ", " << yDes[i] << "), " << aDes[i] << " " << tDes[i] << "\n";
	}*/

	//initalize drone
	droneP = new Drone(0.0, 0.0, 0.0, updateType);
	//droneP = new Drone(0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 1.5, 2.5, 3.5, 1.8, 2.8, 3.8, 1.9, 2.9, 3.9);
	droneP->setMaximums(maxAngle, maxZDot, maxPsiDot);

	//log control gains
	settingsLog << "Controlgains: " << "xPID= " << droneP->_droneController->_kpX << " " << droneP->_droneController->_kdX << " " << droneP->_droneController->_kiX << " yPID= " << droneP->_droneController->_kpY << " " << droneP->_droneController->_kdY << " " << droneP->_droneController->_kiY << " yawPID= " << droneP->_droneController->_kpT << " " << droneP->_droneController->_kdT << " " << droneP->_droneController->_kiT << " zPID= " << droneP->_droneController->_kpA << " " << droneP->_droneController->_kdA << " " << droneP->_droneController->_kiA << "\n";

	//log maximums
	settingsLog << "MaxAngle= " << droneP->_maxAngle << "MaxZDot= " << droneP->_maxZDot << "MaxPsiDot= " << droneP->_maxPsiDot << "\n";
std::cout << "Maximums: " << droneP->_maxAngle << " " << droneP->_maxPsiDot << " " << droneP->_maxZDot << " updateType= " << updateType << std::endl;

//test drone library
droneP->printState();
/*droneP->calculateControl(xDes[0], yDes[0], aDes[0], tDes[0]);
droneP->printState();
droneP->translateControl();
droneP->printState();*/

	/***AR.DRONE TAKEOFF***/
	//command take off
	ROS_INFO("Taking off..");	
	ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
	while (takeoff.getNumSubscribers() < 1) { ;}
	//cmd_vel.publish(velocity);
	//takeoff.publish(empty);
	ros::spinOnce();
	ros::Duration(5).sleep(); // sleep to allow the drone to take off

	int32_t controlFlag = 0;

	/***BEGIN CONTROL LOOP***/
	bool cam = false;
	bool hover = false;
	ros::Time hoverBegin;

	//drive to waypoint only when the drone is more than 0.5m away from the desired waypoint and more than 10degrees off from the desired angle
	double angleThres = 5;
	double distThres = 100; 
	double zThres = 50;
	double hoverTime = 0.5;
	settingsLog << "hoverTime= " << hoverTime << " angleThreshold= " << angleThres << " distThreshold= " << distThres << " zThreshold= " << zThres << "\n";

	ROS_INFO("Begin control loop...");
	ros::Rate loop_rate(200); //200Hz loop rate
	while (ros::ok()) {
	//for (int k = 0; k < 10; k++) {
		//create control signal with default as hover
		geometry_msgs::Twist velocity;
		velocity.linear.x = 0.0;
		velocity.linear.y = 0.0;
		velocity.linear.z = 0.0;	
		velocity.angular.x = 0.0;
		velocity.angular.y = 0.0;
		velocity.angular.z = 0.0;

		//control mode
		//check each individually, but only move to the next waypoint when everything is fulfilled
/*std::cout << "ErrorVals= " << (droneP->_x - xDes[currentWaypoint]) << " " << (droneP->_y - yDes[currentWaypoint]) << " " << droneP->_droneController->angleDiff(droneP->_psi, tDes[currentWaypoint]) << " " << (droneP->_alt - aDes[currentWaypoint]) << std::endl;*/
/*			if ((droneP->_x - xDes[currentWaypoint]) > distThres) {
				ROS_INFO("x not there yet..");
			}
			if ((droneP->_y - yDes[currentWaypoint]) > distThres) {
				ROS_INFO("y not there yet..");
			}
			if (droneP->_droneController->angleDiff(droneP->_psi, tDes[currentWaypoint]) > angleThres) {
				ROS_INFO("angle not there yet..");
			}
			if ((droneP->_alt - aDes[currentWaypoint]) > distThres) {
				ROS_INFO("alt not there yet..");
			}
*/

		if ( ((abs(droneP->_x - xDes[currentWaypoint]) > distThres) || 
		(abs(droneP->_y - yDes[currentWaypoint]) > distThres) ||
		(abs(droneP->_alt - aDes[currentWaypoint]) > zThres) ||
		(abs(droneP->_droneController->angleDiff(droneP->_psi, tDes[currentWaypoint])) > angleThres)) && (hover == false) ) {
		/*if ( ((abs(droneP->_x - xDes[currentWaypoint]) > distThres) || 
		(abs(droneP->_y - yDes[currentWaypoint]) > distThres)) && (hover == false) ) {*/
			//calculate the control signal
			//ROS_INFO("Calculating control signal...");
			droneP->calculateControl(xDes[currentWaypoint], yDes[currentWaypoint], aDes[currentWaypoint], tDes[currentWaypoint]);
		}

		//get elapsed time
   		long end = myclock();
        	long currentTime = (end-programStart)/1000000.0;

		//log control 
		controlLog << "time= " << currentTime << " Waypoint= " << currentWaypoint << " rawErrorVals(x,y,alt,psi)= " << droneP->_droneController->_lastXErr << " " << droneP->_droneController->_lastYErr << " " << droneP->_droneController->_lastAErr << " " << droneP->_droneController->_lastTErr << " rawIntegralErrVals(x,y,alt,psi)= " << droneP->_droneController->_totalXErr << " " << droneP->_droneController->_totalYErr << " " << droneP->_droneController->_totalAErr << " " << droneP->_droneController->_totalTErr << " rawControlSignal= " << droneP->_droneController->_thetaDes << " " << droneP->_droneController->_phiDes << " " << droneP->_droneController->_psiDotDes << " " << droneP->_droneController->_zDotDes;

			droneP->translateControl();

			//log control 
		controlLog << " translatedControlSignal= " << droneP->_droneController->_thetaDes << " " << droneP->_droneController->_phiDes << " " << droneP->_droneController->_psiDotDes << " " << droneP->_droneController->_zDotDes;

		if ( ((abs(droneP->_x - xDes[currentWaypoint]) > distThres) || 
		(abs(droneP->_y - yDes[currentWaypoint]) > distThres) ||
		(abs(droneP->_alt - aDes[currentWaypoint]) > zThres) ||
		(abs(droneP->_droneController->angleDiff(droneP->_psi, tDes[currentWaypoint])) > angleThres)) && (hover == false) ) {
		/*if ( ((abs(droneP->_x - xDes[currentWaypoint]) > distThres) || 
		(abs(droneP->_y - yDes[currentWaypoint]) > distThres)) && (hover == false) ) {*/
			if ( (abs(droneP->_x - xDes[currentWaypoint]) > distThres) ) {
				std::cout << "x control" << std::endl;
				velocity.linear.x = droneP->_droneController->_thetaDes;
			}
			if ( (abs(droneP->_y - yDes[currentWaypoint]) > distThres) ) {
				std::cout << "y control" << std::endl;
				//remember that adrone_autonomy will flip the phides sign
				velocity.linear.y = droneP->_droneController->_phiDes;
			}
	
			if (abs(droneP->_droneController->angleDiff(droneP->_psi, tDes[currentWaypoint])) > angleThres) {
				std::cout << "yaw control" << std::endl;
				velocity.angular.z = droneP->_droneController->_psiDotDes;
			}
			if (abs(droneP->_alt - aDes[currentWaypoint]) > zThres) {
				std::cout << "z control" << std::endl;
				velocity.linear.z = droneP->_droneController->_zDotDes;
			}
		}

		//if already at goal, begin to hover
		else if (hover == false) {
			ROS_INFO("Begin hovering at waypoint %d...", currentWaypoint);
			hoverBegin = ros::Time::now();
			hover = true;

			droneP->_droneController->resetController();
			velocity.linear.x = 0;
			velocity.linear.y = 0;
			velocity.linear.z = 0;	
			velocity.angular.x = 0;
			velocity.angular.y = 0;
			velocity.angular.z = 0;
		}

		//if already hovering, keep hovering for 2 seconds
		else if ((hover == true) && ((ros::Time::now()-hoverBegin).toSec() < hoverTime)) {
			velocity.linear.x = 0;
			velocity.linear.y = 0;
			velocity.linear.z = 0;	
			velocity.angular.x = 0;
			velocity.angular.y = 0;
			velocity.angular.z = 0;
		}

		//if done hovering, move to next goal, or land if last goal is reached
		else {
			hover = false;
			currentWaypoint = currentWaypoint + 1;
			ROS_INFO("Moving to waypoint %d...", currentWaypoint);
			if (currentWaypoint >= numPoints) {
				ROS_INFO("All waypoints visited!");
				break;
			}
		}

		//get elapsed time
   		end = myclock();
        	currentTime = (end-programStart)/1000000.0;

		controlLog << " sentMessageTime= " << ros::Time::now().toSec() << " " << 0 << " sentLinearCommand= " << velocity.linear.x << " " << velocity.linear.y << " " << velocity.linear.z << " " << " sentAngularCommand= " << velocity.angular.x << " " << velocity.angular.y << " " << velocity.angular.z << "\n";

		//send appropriate control signal
		cmd_vel.publish(velocity);
std::cout << " currentWaypoint= " << currentWaypoint << " thetades= " << velocity.linear.x << " phides= " << velocity.linear.y << " X= " << droneP->_x << " Y= " << droneP->_y << " Z= " << droneP->_alt << " psi= " << droneP->_psi << " hover= " << hover << std::endl;
settingsLog << "thetades= " << velocity.linear.x << " phides= " << velocity.linear.y << " X= " << droneP->_x << " Y= " << droneP->_y << " Z= " << droneP->_alt << " psi= " << droneP->_psi << " hover= " << hover << "\n";
		ros::spinOnce();

	}

	//land the quad
	ROS_INFO("Landing..");
	ros::Publisher land = n.advertise<std_msgs::Empty>("/ardrone/land", 1000);
	while (land.getNumSubscribers() < 1) { ;}
	land.publish(empty);
	ros::spinOnce();
	
	//exit cleanly
	cvDestroyWindow("view");
	navLog.close();

	/*//free memory
	droneP->~Drone();
	//delete droneP;
	delete [] xDes;
	delete [] yDes;
	delete [] aDes;
	delete [] tDes; */

	return 0;

}

#endif //_CUSTOMUSER_CPP_
