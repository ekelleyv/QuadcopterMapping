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
#include <limits>

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
#include <rosbag/bag.h>

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

using namespace std;

/*
 * DEFINITIONS
 */
#define MY_MASK 0777 //for setting folder permissions when using mkdir
//path to log files, make sure trailing / is there
#define ROS_WORKSPACE "/home/sytang/Dropbox/control_data/" 

/*
 * GLOBAL VARIABLES
 */
std::fstream settingsLog; //settings of the trial
std::fstream navLog; //name of log file
std::fstream predictLog; //log predicted things
std::fstream controlLog; //log control commands of real drone only
std::fstream edLog; //log particle filter things
std::fstream simLog; //log simulated drone
std::fstream waypointControlLog; //logs waypoint control
std::fstream LFControlLog; //log lf control things
//rosbag::Bag bagLog("bagLog.bag", rosbag::bagmode::Write); //rosbag all messages
int updateType = 0; //0 for navdata, 1 for ptam, 2 for integrate with ptam, 3 for integrate with navdata, 4 for ed's localization code
long programStart; //time of program start
struct tm *now; //beginning of programming
Drone* droneP;
int first = 0;
double yawOffset = 0;

/*
 FUNCTION DECLARATIONS
 */
long myclock();
void navDataCB(const ardrone_autonomy::Navdata::ConstPtr& msg);
void predictDataCB(const tum_ardrone::filter_state::ConstPtr& msg);
void particleFilterCB(const geometry_msgs::Pose::ConstPtr& msg);
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
	//if first navdta, zero the yaw
	/*if (first == 0) {
		first = 1;
		droneP->_yawOffset = droneP->_droneController->angleSum(-msg->rotZ, -180);
	}

	if (abs(droneP->_droneController->angleSum(-msg->rotZ, droneP->_psi)) > 30) {
		droneP->_yawOffset = droneP->_droneController->angleSum(-msg->rotZ, droneP->_psi);
	}*/

	//get elapsed time
   	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	//log all variables
	navLog << "time= " << currentTime << " messageTime= " << msg->header.stamp << " droneTime= " << msg->tm << " lastStateTime= " << droneP->lastStateTime << " batteryPrecent= " << msg->batteryPercent << " state= " << msg->state << " originalPosition= " << droneP->_x << " " << droneP->_y << " rot= " << msg->rotX << " " << msg->rotY << " " << msg->rotZ << " altd= " << msg->altd << " linearV= " << msg->vx << " " << msg->vy << " " << msg->vz << " " << " linearAccel= " << msg->ax << " " << msg->ay << " " << msg->az << " mag= " << msg->magX << " " << msg->magY << " " << msg->magZ << " pressure= " << msg->pressure << " temp= " << msg->temp << " windSpeed= " << msg->wind_speed << " windAngle= " << msg->wind_angle << " windCompAngle= " << msg->wind_comp_angle;

	//bagLog.write("navdata", ros::Time::now(), msg);	
	droneP->updateNavState(msg);
	droneP->updateState();

	navLog << " newPosition= " << droneP->_x << " " << droneP->_y << " angles= " << droneP->_theta << " " << droneP->_phi << " " << droneP->_psi << " altitude= " << droneP->_alt << " velocities= " << droneP->_u << " " << droneP->_v << " " << droneP->_w << " accelerations= " << droneP->_udot << " " << droneP->_vdot << " " << droneP->_wdot << " robot= " << droneP->_index << " yawOffset= " << droneP->_yawOffset << "\n";
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

//	bagLog.write("predictdata", ros::Time::now(), msg);	

	predictLog << " newPosition= " << droneP->_x << " " << droneP->_y << " angles= " << droneP->_theta << " " << droneP->_phi << " " << droneP->_psi << " altitude= " << droneP->_alt << " velocities= " << droneP->_u << " " << droneP->_v << " " << droneP->_w << " accelerations= " << droneP->_udot << " " << droneP->_vdot << " " << droneP->_wdot << "\n";
}

void cmdVelCB(const geometry_msgs::TwistConstPtr& msg) {
	//get elapsed time
   	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	//log control 
	controlLog << "time= " << currentTime << " messageTime = " << ros::Time::now() << " command= " << msg->linear.x << " " << msg->linear.y << " " << msg->linear.z << " " << msg->angular.x << " " << msg->angular.y << " " << msg->angular.z << "\n";
}

/* Callback function to update based on particle filter*/
void particleFilterCB(const geometry_msgs::Pose::ConstPtr& msg) {
	//get elapsed time
   	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	//bagLog.write("PFdata", ros::Time::now(), msg);

	edLog << "time= " << currentTime << " position= " << msg->position.x << " " << msg->position.y << " " << msg->position.z << " quaternion= " << msg->orientation.x << " " <<msg->orientation.y << " " <<  msg->orientation.z << " " << msg->orientation.w;
//quaternion is about [0,0,1]

	droneP->updatePFState(msg);
	droneP->updateState();
	edLog << " newPosition= " << droneP->_x << " " << droneP->_y << " angles= " << droneP->_theta << " " << droneP->_phi << " " << droneP->_psi << " altitude= " << droneP->_alt << " velocities= " << droneP->_u << " " << droneP->_v << " " << droneP->_w << " accelerations= " << droneP->_udot << " " << droneP->_vdot << " " << droneP->_wdot << "\n";

}

/*Log simulated drone states*/
void logSimStates(Drone* droneToLog, int r) {
	//get elapsed time
   	long end = myclock();
        long currentTime = (end-programStart)/1000000.0;

	simLog << "time= " << currentTime << " robot= " << r << " newPosition= " << droneToLog->_x << " " << droneToLog->_y << " angles= " << droneToLog->_theta << " " << droneToLog->_phi << " " << droneToLog->_psi << " altitude= " << droneToLog->_alt << " velocities= " << droneToLog->_u << " " << droneToLog->_v << " " << droneToLog->_w << " accelerations= " << droneToLog->_udot << " " << droneToLog->_vdot << " " << droneToLog->_wdot << " angleVelocities= " << droneToLog->_thetadot << " " << droneToLog->_phidot << " " << droneToLog->_psidot << " psiddot= " << droneToLog->_psiddot << "\n";
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
	processCmdLine(argc, argv, &maxAngle, &maxZDot, &maxPsiDot, &updateType);

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
	settingsLog.open(settingsStr.str().c_str(), std::ios_base::out);

	std::stringstream navLogStr;
	navLogStr << ROS_WORKSPACE << "navLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	navLog.open(navLogStr.str().c_str(), std::ios_base::out);

	std::stringstream predictLogStr;
	predictLogStr << ROS_WORKSPACE << "predictLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	predictLog.open(predictLogStr.str().c_str(), std::ios_base::out);

	std::stringstream controlLogStr;
	controlLogStr << ROS_WORKSPACE << "controlLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	controlLog.open(controlLogStr.str().c_str(), std::ios_base::out);

	std::stringstream edLogStr;
	edLogStr << ROS_WORKSPACE << "edLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	edLog.open(edLogStr.str().c_str(), std::ios_base::out);

	std::stringstream simLogStr;
	simLogStr << ROS_WORKSPACE << "simLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	simLog.open(simLogStr.str().c_str(), std::ios_base::out);

	std::stringstream LFControlLogStr;
	LFControlLogStr << ROS_WORKSPACE << "lfControlLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	LFControlLog.open(LFControlLogStr.str().c_str(), std::ios_base::out);

	std::stringstream waypointControlLogStr;
	waypointControlLogStr << ROS_WORKSPACE << "waypointControlLog" << now->tm_mon+1 << "_" << now->tm_mday<< "_" << now->tm_year+1900 << "_" << now->tm_hour << "_" << now->tm_min << ".txt";
	waypointControlLog.open(waypointControlLogStr.str().c_str(), std::ios_base::out);




	std_msgs::Empty empty;
	std_srvs::Empty emptyCall;

	/***SET UP SERVICES***/
	//subscribe to camera feeds
	image_transport::ImageTransport it(n);
	image_transport::Subscriber frontCam_sub = it.subscribe("/ardrone/front/image_raw", 1, imgCB); //forward camera
	//image_transport::Subscriber bottomCam_sub = it.subscribe("/ardrone/bottom/image_raw", 1, imgCB); //downward camera

	//service call to toggle camera
	ros::ServiceClient toggleCam = n.serviceClient<std_srvs::Empty>("/ardrone/toggleCam");

	//subscribe to state estimates
	ros::Subscriber navdata_sub = n.subscribe("/ardrone/navdata", 1000, navDataCB);
	ros::Subscriber predictdata_sub = n.subscribe("/ardrone/predictedPose", 1000, predictDataCB);
	ros::Subscriber particlefilter_sub = n.subscribe("/pf_localization", 1000, particleFilterCB);

	/***INITIALIZE DRONE STUFF***/
	ROS_INFO("Initialize drone stuff...");
	//initialize desired waypoints
	int numWaypoints;
	int currentWaypoint = 0;
	//assume units of mm
	std::vector<double> xDes;
	std::vector<double> yDes;
	std::vector<double> aDes;
	std::vector<double> tDes;
	std::vector<double> hoverTime;

	tasksLoadFile("waypoints.txt", &xDes, &yDes, &aDes, &tDes, &hoverTime, &numWaypoints); 


	//log waypoints
	settingsLog << " NumberWaypoints= " << numWaypoints << " xDes= ";
	for (int i = 0; i < numWaypoints; i++) {
		settingsLog << xDes[i] << " ";
	} 
	settingsLog << "yDes= ";
	for (int i = 0; i < numWaypoints; i++) {
		settingsLog << yDes[i] << " ";
	}
	settingsLog << " altDes= ";
	for (int i = 0; i < numWaypoints; i++) {
		settingsLog << aDes[i] << " ";
	}
	settingsLog << " psiDes= ";
	for (int i = 0; i < numWaypoints; i++) {
		settingsLog << tDes[i] << " ";
	}
	for (int i = 0; i < numWaypoints; i++) {
		settingsLog << hoverTime[i] << " ";
	}
	settingsLog << "\n";

for (int i = 0; i < numWaypoints; i++) {
		std::cout << "(" << xDes[i] << ", " << yDes[i] << "), " << aDes[i] << " " << tDes[i] << " time= " << hoverTime[i] << "\n";
	}

	//initalize drone
	int numRobots = 1;
	std::vector<double> initx;
	std::vector<double> inity;	
	std::vector<double> initz;
	std::vector<int> droneType;
	std::vector<int> updateType;
	std::vector<Drone*> allRobots;

	dronesLoadFile("drones.txt", &initx, &inity, &initz, &updateType, &droneType, &numRobots);

	for (int r = 0; r < numRobots; r++) {

		Drone* thisDrone = new Drone(initx[r], inity[r], initz[r], updateType[r], droneType[r], r);
		allRobots.push_back(thisDrone);
		allRobots[r]->setMaximums(maxAngle, maxZDot, maxPsiDot);
		if (droneType[r] == 1) { droneP = thisDrone; }
	}

	for (int r = 0; r < numRobots; r++) {
		allRobots[r]->printState();
	}

	//log control gains
	for (int r = 0; r < numRobots; r ++) { 
		Drone* thisDrone = allRobots[r];
		settingsLog << "Robot= " << r << " robotType= " << thisDrone->_droneType << " Controlgains:" << " xPID= " << thisDrone->_droneController->_kpX << " " << thisDrone->_droneController->_kdX << " " << thisDrone->_droneController->_kiX << " yPID= " << thisDrone->_droneController->_kpY << " " << thisDrone->_droneController->_kdY << " " << thisDrone->_droneController->_kiY << " yawPID= " << thisDrone->_droneController->_kpT << " " << thisDrone->_droneController->_kdT << " " << thisDrone->_droneController->_kiT << " zPID= " << thisDrone->_droneController->_kpA << " " << thisDrone->_droneController->_kdA << " " << thisDrone->_droneController->_kiA << " lGammaControl= " << thisDrone->_droneFController->_ka1 << " " << thisDrone->_droneFController->_ka2 << " llControl= " << thisDrone->_droneFController->_kc1 << " " << thisDrone->_droneFController->_kc2 << " MaxAngle= " << thisDrone->_maxAngle << " MaxZDot= " << thisDrone->_maxZDot << " MaxPsiDot= " << thisDrone->_maxPsiDot << " updateType= " << thisDrone->_updateType << "\n";
	}


	//get desired formation
	std::vector< std::vector<double> > Fl;
	std::vector< std::vector<double> > Fgamma;

	formationLoadFile("formation.txt", &Fl);
	settingsLog << "Formation: \n";
	for (int r = 0; r < numRobots; r++) {
		for (int s = 0; s < numRobots; s++) {
			if (r == s) { Fl[r][s] = std::numeric_limits<int>::max();}
			settingsLog << Fl[r][s] << " ";
			std::cout << Fl[r][s] << " ";
		}
		settingsLog << "\n";
		std::cout << "\n";
	}

	formationGammaLoadFile("formationGamma.txt", &Fgamma);
	settingsLog << "FormationGamma: \n"; 
	for (int r = 0; r < numRobots; r++) {
		for (int s = 0; s < numRobots; s++) {
			if (r == s) { Fgamma[r][s] = std::numeric_limits<int>::max();}
			settingsLog << Fgamma[r][s] << " ";
			std::cout << Fgamma[r][s] << " ";
		}
		settingsLog << "\n";
		std::cout << "\n";
	}







	/***AR.DRONE TAKEOFF***/
	ROS_INFO("Initializing openCV...");
	//make openCV windows
	cvNamedWindow("view");
  	cvStartWindowThread();



	ROS_INFO("Zeroing cmd_vel..");
	//initialize cmd_vel
	ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	while (cmd_vel.getNumSubscribers() < 1) { ;}
	geometry_msgs::Twist velocity;
	velocity.linear.x = 0;
	velocity.linear.y = 0;
	velocity.linear.z = 0;	
	velocity.angular.x = 0;
	velocity.angular.y = 0;
	velocity.angular.z = 0;
	cmd_vel.publish(velocity);
	//ros::spinOnce(); //reset velocity
	//ros::Subscriber cmdvel_sub = n.subscribe("/cmd_vel", 1000, cmdVelCB);

	bool cam = true; //true if facing forward
	//if (toggleCam.call(emptyCall)) { ROS_INFO("Toggling camera");}
	//else { ROS_ERROR("Failed to call toggle camera service");}//toggle the downward camera
	//cam = false;

	//command take off
	ROS_INFO("Taking off..");	
	ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
	while (takeoff.getNumSubscribers() < 1) { ;}
	cmd_vel.publish(velocity);
	takeoff.publish(empty);
	ros::spinOnce();
	ros::Duration(10).sleep(); // sleep to allow the drone to take off
	int32_t controlFlag = 0;

	/***BEGIN CONTROL LOOP***/
	bool hover = false;
	ros::Time hoverBegin;

	//drive to waypoint only when the drone is more than 0.5m away from the desired waypoint and more than 10degrees off from the desired angle
	double angleThres_ctrl = 5;
	double distThres_ctrl = 100; 
	double zThres_ctrl = 50;
	double angleThres_hover = 10;
	double distThres_hover = 200;
	double zThres_hover = 50;
	double lThres_ctrl = 150;
	double gammaThres_ctrl = 10;
	double currentHoverTime = hoverTime[currentWaypoint];
	settingsLog << " angleThreshold= " << angleThres_hover << " " << angleThres_ctrl << " distThreshold= " << distThres_hover << " " << distThres_ctrl << " zThreshold= " << zThres_hover << " " <<zThres_ctrl << "\n";

	ROS_INFO("Begin control loop...");
	ros::Rate loop_rate(200); //200Hz loop rate
	
	while (ros::ok()) {

		//create control signal with default as hover
		geometry_msgs::Twist velocity;
		velocity.linear.x = 0.0;
		velocity.linear.y = 0.0;
		velocity.linear.z = 0.0;	
		velocity.angular.x = 0.0;
		velocity.angular.y = 0.0;
		velocity.angular.z = 0.0;

		double commandLinearX = 0;
		double commandLinearY = 0;
		double commandLinearZ = 0;
		double commandAngularZ = 0;




		// PID WAYPOINT CONTROL
		Drone* thisDrone = allRobots[0]; //first robot always global leader

		//calcualte control
		thisDrone->calculateControl(xDes[currentWaypoint], yDes[currentWaypoint], aDes[currentWaypoint], tDes[currentWaypoint]);

		//get elapsed time
   		long end = myclock();
       		long currentTime = (end-programStart)/1000000.0;

	
		//log control 
		waypointControlLog << "time= " << currentTime << " Waypoint= " << currentWaypoint << " rawErrorVals(x,y,alt,psi)= " << thisDrone->_droneController->_lastXErr << " " << thisDrone->_droneController->_lastYErr << " " << thisDrone->_droneController->_lastAErr << " " << thisDrone->_droneController->_lastTErr << " rawIntegralErrVals(x,y,alt,psi)= " << thisDrone->_droneController->_totalXErr << " " << thisDrone->_droneController->_totalYErr << " " << thisDrone->_droneController->_totalAErr << " " << thisDrone->_droneController->_totalTErr << " rawControlSignal= " << thisDrone->_droneController->_thetaDes << " " << thisDrone->_droneController->_phiDes << " " << thisDrone->_droneController->_psiDotDes << " " << thisDrone->_droneController->_zDotDes;

		//translate control to being between -1 and 1
		thisDrone->translateControl();

		//log control 
		waypointControlLog << " translatedControlSignal= " << thisDrone->_droneController->_thetaDes << " " << thisDrone->_droneController->_phiDes << " " << thisDrone->_droneController->_psiDotDes << " " << thisDrone->_droneController->_zDotDes;


		//populate command vel with the control signals you actually want
		if ( (abs(thisDrone->_x - xDes[currentWaypoint]) > distThres_ctrl) || (abs(thisDrone->_y - yDes[currentWaypoint]) > distThres_ctrl) ) {
				//limit controller for debugging
//ROS_INFO("position not there yet..");
			commandLinearX = thisDrone->_droneController->_thetaDes; 				
			commandLinearY = thisDrone->_droneController->_phiDes; //remember ardrone will flip this sign
		}
	
		if (abs(thisDrone->_droneController->angleDiff(thisDrone->_psi, tDes[currentWaypoint])) > angleThres_ctrl) {
//ROS_INFO("yaw not there yet..");
			commandAngularZ = thisDrone->_droneController->_psiDotDes;
		}
		if (abs(thisDrone->_alt - aDes[currentWaypoint]) > zThres_ctrl) {
//ROS_INFO("alt not there yet..");
			commandLinearZ = thisDrone->_droneController->_zDotDes;
		}


		//if at goal, begin to hover
		if ( ((abs(thisDrone->_x - xDes[currentWaypoint]) < distThres_hover) && 
		(abs(thisDrone->_y - yDes[currentWaypoint]) < distThres_hover) &&
		(abs(thisDrone->_alt - aDes[currentWaypoint]) < zThres_hover) &&
		(abs(thisDrone->_droneController->angleDiff(thisDrone->_psi, tDes[currentWaypoint])) < angleThres_hover)) && (hover == false) ) {
			ROS_INFO("Begin hovering at waypoint %d...", currentWaypoint);
			hoverBegin = ros::Time::now();
			hover = true;

		}

		//if already hovering, but moves out of range, reset timer
		if ( ((abs(thisDrone->_x - xDes[currentWaypoint]) > distThres_hover) || 
		(abs(thisDrone->_y - yDes[currentWaypoint]) > distThres_hover) ||
		(abs(thisDrone->_alt - aDes[currentWaypoint]) > zThres_hover) ||
		(abs(thisDrone->_droneController->angleDiff(thisDrone->_psi, tDes[currentWaypoint])) > angleThres_hover)) && (hover == true) ) {
			hover = false;
		}

		//if done hovering, move to next goal, or land if last goal is reached
		if ((hover == true) && ((ros::Time::now()-hoverBegin).toSec() > currentHoverTime)) {
			hover = false;
			currentWaypoint = currentWaypoint + 1;
			currentHoverTime = hoverTime[currentWaypoint];
			ROS_INFO("Moving to waypoint %d...", currentWaypoint);

			thisDrone->_droneController->resetController();
			commandLinearX = 0;
			commandLinearY = 0;
			commandLinearZ = 0;	
			commandAngularZ = 0;

			if (currentWaypoint >= numWaypoints) {
				ROS_INFO("All waypoints visited!");
				break;
			}
		}

		//get elapsed time
   		end = myclock();
        	currentTime = (end-programStart)/1000000.0;

		waypointControlLog << " sentMessageTime= " << currentTime << " " << 0 << " sentLinearCommand= " << commandLinearX << " " << commandLinearY << " " << commandLinearZ << " " << " sentAngularCommand= " << 0 << " " << 0 << " " << commandAngularZ << " robot= 0 \n";
		//bagLog.write("controlCommands", ros::Time::now(), velocity);	


		if (allRobots[0]->_droneType == 1) {
		velocity.linear.x = commandLinearX;
		velocity.linear.y = commandLinearY;
		velocity.linear.z = commandLinearZ;
		velocity.angular.z = commandAngularZ;

		//command actual drone
		cmd_vel.publish(velocity);

		controlLog << "robot= 0" << " sentMessageTime= " << currentTime << " sentLinearCommand= " << velocity.linear.x << " " << velocity.linear.y << " " << velocity.linear.z << " " << " sentAngularCommand= " << velocity.angular.x << " " << velocity.angular.y << " " << velocity.angular.z << "\n";

		std::cout << "state= " << hover << " point= " << currentWaypoint << " Pos=(" << thisDrone->_x << ", " << thisDrone->_y << ", " << thisDrone->_alt << ", " << thisDrone->_psi << ") desiredAng=(" << commandLinearX << ", " << commandLinearY  << ", " << commandLinearZ << ", " << commandAngularZ << ") GlobalErr=(" << xDes[currentWaypoint] - thisDrone->_x << ", " << yDes[currentWaypoint] - thisDrone->_y << ", " << aDes[currentWaypoint] - thisDrone->_alt << ", " << thisDrone->_droneController->angleDiff(thisDrone->_psi, tDes[currentWaypoint]) << ") Localerr=(" << thisDrone->_droneController->_lastXErr << ", " << thisDrone->_droneController->_lastYErr << ", " << thisDrone->_droneController->_lastAErr << ", " << thisDrone->_droneController->_lastTErr << ")" << std::endl;
		//ros::spinOnce();
		}
		else if (allRobots[0]->_droneType == 0) {

		//remember to flip v command	
		//thisDrone->simulateCommands(commandLinearX*thisDrone->_maxAngle, - commandLinearY*thisDrone->_maxAngle, commandAngularZ*thisDrone->_maxPsiDot, commandLinearZ*thisDrone->_maxZDot);
		//thisDrone->simulateDrone();
		logSimStates(thisDrone, 0);
		}




		//LEADER-FOLLOWER CONTROL
		//robot 0 already taken care of
		for (int r = 1; r < numRobots; r++) {
		commandLinearX = 0;
		commandLinearY = 0;
		commandLinearZ = 0;
		commandAngularZ = 0;


			Drone* thisDrone = allRobots[r];
			allRobots[r]->simulateControlGraph(Fl, Fgamma, r, numRobots, allRobots);
			thisDrone->calculateFormationControl();

			LFControlLog << " controlType= " << thisDrone->_droneFController->_controlType << " leaders= " << thisDrone->_droneFController->_leader1 << " " << thisDrone->_droneFController->_leader2 << " desiredl= " << thisDrone->_droneFController->_l1des << " " << thisDrone->_droneFController->_l2des << " desiredGamma= " << thisDrone->_droneFController->_gamma1des << " leader1states= " << thisDrone->_droneFController->_xl1 << " " << thisDrone->_droneFController->_yl1 << " " << thisDrone->_droneFController->_psil1 << " leader2states= " << thisDrone->_droneFController->_xl2 << " " << thisDrone->_droneFController->_yl2 << " " << thisDrone->_droneFController->_psil2 << " rawControlSignal= " << thisDrone->_droneController->_thetaDes << " " << thisDrone->_droneController->_phiDes << " " << thisDrone->_droneController->_psiDotDes << " " << thisDrone->_droneController->_zDotDes;


		thisDrone->translateLFControl();

		//log control 
		LFControlLog << " translatedControlSignal= " << thisDrone->_droneFController->_thetaDes << " " << thisDrone->_droneFController->_phiDes << " " << thisDrone->_droneFController->_psiDotDes << " " << thisDrone->_droneFController->_zDotDes;


		double error1;
		double error2;

		if (thisDrone->_droneFController->_controlType == 0) {
		error1 = sqrt( (thisDrone->_x-thisDrone->_droneFController->_xl1)*(thisDrone->_x-thisDrone->_droneFController->_xl1) + (thisDrone->_y-thisDrone->_droneFController->_yl1)*(thisDrone->_y-thisDrone->_droneFController->_yl1));
		error1 = error1 - thisDrone->_droneFController->_l1des;
		error2 = atan2( thisDrone->_y - thisDrone->_droneFController->_yl1, thisDrone->_x - thisDrone->_droneFController->_xl1 );
		error2 = thisDrone->_droneFController->angleSum(error2*180/M_PI, -thisDrone->_droneFController->_psil1);
		error2 = thisDrone->_droneFController->angleSum(error2, 180);
		error2 = thisDrone->_droneFController->angleSum(error2, -thisDrone->_droneFController->_gamma1des);
		}
		else if (thisDrone->_droneFController->_controlType == 1) {
		error1 = sqrt( (thisDrone->_x-thisDrone->_droneFController->_xl1)*(thisDrone->_x-thisDrone->_droneFController->_xl1) + (thisDrone->_y-thisDrone->_droneFController->_yl1)*(thisDrone->_y-thisDrone->_droneFController->_yl1));
		error1 = error1 - thisDrone->_droneFController->_l1des;
		error2 = sqrt( (thisDrone->_x-thisDrone->_droneFController->_xl2)*(thisDrone->_x-thisDrone->_droneFController->_xl2) + (thisDrone->_y-thisDrone->_droneFController->_yl2)*(thisDrone->_y-thisDrone->_droneFController->_yl2));
		error2 = error2 - thisDrone->_droneFController->_l2des;
		}

		if ( (abs(error1) > lThres_ctrl) || (abs(error2) > gammaThres_ctrl) ) {
		commandLinearX = thisDrone->_droneFController->_thetaDes;
		commandLinearY = thisDrone->_droneFController->_phiDes;
		commandLinearZ = thisDrone->_droneFController->_zDotDes;
		commandAngularZ = thisDrone->_droneFController->_psiDotDes;
		}


		//try to keep heading on track
		thisDrone->calculateControl(0, 0, 0, -180);
		thisDrone->translateControl();

		if (abs(thisDrone->_droneController->angleDiff(thisDrone->_psi, tDes[currentWaypoint])) > angleThres_ctrl) { commandAngularZ = thisDrone->_droneController->_psiDotDes;}



		//get elapsed time
   		end = myclock();
        	currentTime = (end-programStart)/1000000.0;

		LFControlLog << " sentMessageTime= " << currentTime << " " << 0 << " sentLinearCommand= " << commandLinearX << " " << commandLinearY << " " << commandLinearZ << " " << " sentAngularCommand= " << 0 << " " << 0 << " " << commandAngularZ << " robot= " << r << " errors= " << error1 << " " << error2 << "\n";

   		end = myclock();
        	currentTime = (end-programStart)/1000000.0;

		if (allRobots[r]->_droneType == 0) {

		//thisDrone->simulateCommands(commandLinearX*thisDrone->_maxAngle, - commandLinearY*thisDrone->_maxAngle, commandAngularZ*thisDrone->_maxPsiDot, commandLinearZ*thisDrone->_maxZDot);
		//thisDrone->simulateDrone();
		logSimStates(thisDrone, r);
		}
		else if (allRobots[r]->_droneType == 1) {

		velocity.linear.x = commandLinearX;
		velocity.linear.y = commandLinearY;
		velocity.linear.z = commandLinearZ;
		velocity.angular.z = commandAngularZ;

		//command actual drone
		cmd_vel.publish(velocity);

		controlLog << "robot= 0" << " sentMessageTime= " << currentTime << " sentLinearCommand= " << velocity.linear.x << " " << velocity.linear.y << " " << velocity.linear.z << " " << " sentAngularCommand= " << velocity.angular.x << " " << velocity.angular.y << " " << velocity.angular.z << "\n";

		std::cout << " lfcontroller= " << thisDrone->_droneFController->_controlType << " Position=(" << thisDrone->_x << ", " << thisDrone->_y << ", " << thisDrone->_alt << ", " << thisDrone->_psi << ") angdesiredAngles=(" << commandLinearX << ", " << commandLinearY  << ", " << commandLinearZ << ", " << commandAngularZ << ") Errors=(" << error1 << ", " << error2 << ")" << std::endl;
		//ros::spinOnce();		
		}
	}

	ros::spinOnce();


/*
		//LEADER-FOLLOWER CONTROL
		//hardcode this for now
		droneP->_droneFController->_controlType = 0;
		droneP->_droneFController->_leader1 = 1;
		droneP->_droneFController->_l1des = sqrt(8);
		droneP->_droneFController->_gamma1des = 135;
		droneP->_droneFController->_xl1 = 0;
		droneP->_droneFController->_yl1 = 0;
		droneP->_droneFController->_psil1 = -180;

		droneP->_droneFController->_controlType = 1;
		droneP->_droneFController->_leader1 = 0;
		droneP->_droneFController->_leader2 = 1;
		droneP->_droneFController->_l1des = sqrt(2);
		droneP->_droneFController->_l2des = sqrt(2);
		droneP->_droneFController->_xl1 = 0;
		droneP->_droneFController->_yl1 = 0;
		droneP->_droneFController->_psil1 = -90;
		droneP->_droneFController->_xl2 = -2;
		droneP->_droneFController->_yl2 = 0;
		droneP->_droneFController->_psil2 = -90;


		droneP->calculateFormationControl();
		//get elapsed time
   		end = myclock();
        	currentTime = (end-programStart)/1000000.0;

		LFControlLog << "time= " << " controlType= " << droneP->_droneFController->_controlType << " leaders= " << droneP->_droneFController->_leader1 << " " << droneP->_droneFController->_leader2 << " desiredl= " << droneP->_droneFController->_l1des << " " << droneP->_droneFController->_l2des << " desiredGamma= " << droneP->_droneFController->_gamma1des << " leader1states= " << droneP->_droneFController->_xl1 << " " << droneP->_droneFController->_yl1 << " " << droneP->_droneFController->_psil1 << " leader2states= " << droneP->_droneFController->_xl2 << " " << droneP->_droneFController->_yl2 << " " << droneP->_droneFController->_psil2 << " rawControlSignal= " << droneP->_droneController->_thetaDes << " " << droneP->_droneController->_phiDes << " " << droneP->_droneController->_psiDotDes << " " << droneP->_droneController->_zDotDes;


		droneP->translateControl();

		//log control 
		LFControlLog << " translatedControlSignal= " << droneP->_droneFController->_thetaDes << " " << droneP->_droneFController->_phiDes << " " << droneP->_droneFController->_psiDotDes << " " << droneP->_droneFController->_zDotDes;

		double l12 = sqrt( (droneP->_x-droneSim->_x)*(droneP->_x-droneSim->_x) + (droneP->_y-droneSim->_y)*(droneP->_y-droneSim->_y));
		double gamma12 = atan2( droneP->_y - droneSim->_y, droneSim->_x -droneSim->_x );
		gamma12 = droneP->_droneFController->angleSum(gamma12*180/M_PI, -droneSim->_psi);
		gamma12 = droneP->_droneFController->angleSum(gamma12, 3.14);

std::cout << " lfcontroller= " << droneP->_droneFController->_controlType << " Position=(" << droneP->_x << ", " << droneP->_y << ", " << droneP->_alt << ", " << droneP->_psi << ") desiredAngles=(" << droneP->_droneFController->_thetaDes << ", " << droneP->_droneFController->_phiDes  << ", " << droneP->_droneFController->_zDotDes << ", " << droneP->_droneFController->_psiDotDes << ") Errors=(" << l12 << ", " << gamma12 << ")" << std::endl;

		droneP->simulateCommands(droneP->_droneFController->_thetaDes*droneP->_maxAngle, -droneP->_droneFController->_phiDes*droneP->_maxAngle, droneP->_droneFController->_psiDotDes*droneP->_maxPsiDot, droneP->_droneFController->_zDotDes*droneP->_maxZDot);
		droneP->simulateDrone();*/
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
	settingsLog.close();
	predictLog.close();
	controlLog.close();
	LFControlLog.close();
	//bagLog.close();

	//free memory
	//droneP->~Drone();
	//delete droneP;

	return 0;

}

#endif //_CUSTOMUSER_CPP_
