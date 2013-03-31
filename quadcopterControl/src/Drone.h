#ifndef _DRONE_H
#define _DRONE_H
/*
 Drone.h
 Data type for the drone
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */


/*
 *QUADCOPTER_CONTROL INCLUDES
 */
#include "Controller.h"
#include "ardrone_autonomy/Navdata.h"

/* 
 * C++ INCLUDES
 */
#include <algorithm>

class Drone {

public: 
	//CONSTRUCTORS
	Drone(double initialx, double initialy, double initialalt);
	Drone(double initialx, double initialy, double initialalt, double kpX, double kiX, double kdX, double kpY, double kiY, double kdY, double kpA, double kiA, double kdA, double kpT, double kiT, double kdT);

	//DESTRUCTOR
	~Drone();

	//PUBLIC FUNCTIONS
	void updateState(const ardrone_autonomy::Navdata::ConstPtr& msg);
	void printState();
	void setMaximums(double maxAngle, double maxZDot, double maxPsiDot);
	void calculateControl(double desX, double desY, double desA, double desT);
	void translateControl();

//private:
	//drone state
	//in millimeters and degrees
	double _x;
	double _y;
	double _alt;
	double _u;
	double _v;
	double _w;
	double _udot;
	double _vdot;
	double _wdot;
	double _theta;
	double _phi;
	double _psi;

	double _maxAngle;
	double _maxZDot;
	double _maxPsiDot;

	ros::Time lastStateTime;
	ros::Time lastControlTime;
	
	Controller* _droneController;
};



#endif
