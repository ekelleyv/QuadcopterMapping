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
 * TUM_ARDRONE INCLUDES
 */
#include "tum_ardrone/filter_state.h"
 
/* 
 *  ROS INCLUDES
 * 
 */
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

/* 
 * C++ INCLUDES
 */
#include <algorithm>

class Drone {

public: 
	//CONSTRUCTORS
	Drone(double initialx, double initialy, double initialalt, int updateType);
	Drone(double initialx, double initialy, double initialalt, double kpX, double kiX, double kdX, double kpY, double kiY, double kdY, double kpA, double kiA, double kdA, double kpT, double kiT, double kdT, int updateType);

	//DESTRUCTOR
	~Drone();

	//PUBLIC FUNCTIONS
	void updateNavState(const ardrone_autonomy::Navdata::ConstPtr& msg);
	void updateTumState(const tum_ardrone::filter_state::ConstPtr& msg);
	void updatePFState(const geometry_msgs::Pose::ConstPtr& msg);
	void updateState();
	void printState();
	void setMaximums(double maxAngle, double maxZDot, double maxPsiDot);
	void calculateControl(double desX, double desY, double desA, double desT);
	void calculateXControl(double desX, double desY);
	void calculateYControl(double desX, double desY);
	void calculateAControl(double desA);
	void calculateYawControl(double desT);
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

	double _yawOffset;

	//navdata data
	double _xn;
	double _yn;
	double _altn;
	double _un;
	double _vn;
	double _wn;
	double _udotn;
	double _vdotn;
	double _wdotn;
	double _thetan;
	double _phin;
	double _psin;

	//particle filter data
	double _xpf;
	double _ypf;
	double _zpf;
	geometry_msgs::Quaternion _orient;

	//tum_ardrone data
	double _xt;
	double _yt;
	double _altt;
	double _ut;
	double _vt;
	double _wt;
	double _thetat;
	double _phit;
	double _psit;

	double _maxAngle;
	double _maxZDot;
	double _maxPsiDot;

	ros::Time lastStateTime;
	ros::Time lastControlTime;
	ros::Time lastUpdateTime; 
	ros::Time lastTumTime;
	ros::Time lastPFTime;

	Controller* _droneController;
	int _updateType;
};



#endif
