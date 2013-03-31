#ifndef _DRONE_CPP_
#define _DRONE_CPP_
/*
 Drone.cpp
 Data type for the drone
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */

/*C++ INCLUDES*/
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <algorithm>
using namespace std;

/*
 *QUADCOPTER_CONTROL INCLUDES
 */
#include "Drone.h"
#include "Controller.h"


/*Drone constructor */
Drone::Drone(double initialx, double initialy, double initialalt) {
	_x = initialx;
	_y = initialy;
	_alt = initialalt;

	_u = 0;
	_v = 0;
	_w = 0;
	_udot = 0;
	_vdot = 0;
	_wdot = 0;
	_theta = 0;
	_phi = 0;
	_psi = 0;

	_maxAngle = 12;
	_maxZDot = 700;
	_maxPsiDot = 100;

	lastStateTime = ros::Time::now();
	lastControlTime = ros::Time::now();

	_droneController = new Controller();
}


Drone::Drone(double initialx, double initialy, double initialalt, double kpX, double kiX, double kdX, double kpY, double kiY, double kdY, double kpA, double kiA, double kdA, double kpT, double kiT, double kdT) {

	_x = initialx;
	_y = initialy;
	_alt = initialalt;

	_u = 0;
	_v = 0;
	_w = 0;
	_udot = 0;
	_vdot = 0;
	_wdot = 0;
	_theta = 0;
	_phi = 0;
	_psi = 0;

	lastStateTime = ros::Time::now();
	lastControlTime = ros::Time::now();

	_droneController = new Controller(kpX, kiX, kdX, kpY, kiY, kdY, kpA, kiA, kdA, kpT, kiT, kdT);

}

/* Drone destructor*/
Drone::~Drone(){
	_droneController->~Controller();
};

/*Update the state of the drone*/
void Drone::updateState(const ardrone_autonomy::Navdata::ConstPtr& msg) {
	//msg->state - use drone state somehow?

	//use dead reckoning for position estimates
	//rotate global position into the drone's frame
	double dt = (msg->header.stamp - lastStateTime).toSec();
	lastStateTime = msg->header.stamp;
	_x = _x + ((_u*cos(_psi*M_PI/180)-_v*sin(_psi*M_PI/180))*dt);
	_y = _y + ((_u*sin(_psi*M_PI/180)+_v*cos(_psi*M_PI/180))*dt);

	_u = msg->vx;
	_v = msg->vy;
	_w = msg->vz;
	_udot = msg->ax;
	_vdot = msg->ay;
	_wdot = msg->az;
	_phi = msg->rotX;
	_theta =  msg->rotY;
	_psi = msg->rotZ;
	_alt = msg->altd;


}


void Drone::printState() {

	std::cout << "\nPosition= " << _x << ", " << _y << ", " << _alt << "; LinearSpeed= " << _u << ", " << _v << ", " << _w << "; LinearAcceleration= " << _udot << ", " << _vdot << ", " << _wdot << "; Angles= " << _theta << ", " << _phi << ", " << _psi << std::endl;
	std::cout << " LastStateTime= " << lastStateTime << " lastControlTime= " << lastControlTime << std::endl;
	std::cout << " Controller= " << std::endl;
	_droneController->printState();
}

/*Set maximum limits for the drone*/
void Drone::setMaximums(double maxAngle, double maxZDot, double maxPsiDot) {
	if (maxAngle > -1) { _maxAngle = maxAngle;}
	if (maxZDot > -1) { _maxZDot = maxZDot;}
	if (maxPsiDot > -1) { _maxPsiDot = maxPsiDot;}
}

/*Calculate the appropriate control signal*/
void Drone::calculateControl(double desX, double desY, double desA, double desH) {
	double dt = (ros::Time::now() - lastControlTime).toSec();
	lastControlTime = ros::Time::now();

	double eXGlobal = desX-_x;
	double eYGlobal = desY-_y;
	double eX = (eXGlobal*cos(_psi*M_PI/180)) + (eYGlobal*sin(_psi*M_PI/180));
	double eY = (-eXGlobal*sin(_psi*M_PI/180)) + (eYGlobal*cos(_psi*M_PI/180));
	double eA = desA-_alt;
	double eT = _droneController->angleDiff(desH, _psi);

	//update integral error
	_droneController->_totalXErr = _droneController->_totalXErr + eX;
	_droneController->_totalYErr = _droneController->_totalYErr + eY;
	_droneController->_totalAErr = _droneController->_totalAErr + eA;
	_droneController->_totalTErr = _droneController->_totalTErr + eT;

	//calculate PID signal
	_droneController->_thetaDes = (_droneController->_kpX*eX) + (_droneController->_kiX*_droneController->_totalXErr) + (_droneController->_kdX*_u);
	_droneController->_phiDes = (_droneController->_kpY*eY) + (_droneController->_kiY*_droneController->_totalYErr) + (_droneController->_kdY*_v);
	_droneController->_psiDotDes = (_droneController->_kpT*eT) + (_droneController->_kiT*_droneController->_totalTErr) + (_droneController->_kdT*(_droneController->_lastTErr - eT)/dt);
	_droneController->_zDotDes = (_droneController->_kpA*eA) + (_droneController->_kiA*_droneController->_totalAErr) + (_droneController->_kdA*(_droneController->_lastAErr - eA)/dt);
	
	//update last error
	_droneController->_lastXErr = eX;
	_droneController->_lastYErr = eY;
	_droneController->_lastAErr = eA;
	_droneController->_lastTErr = eT;

} 

/*Transate control signals to be between -1 and 1 of maximum*/
void Drone::translateControl() {
	//limit to maximums
	_droneController->_thetaDes = min(_maxAngle, max(-_maxAngle, _droneController->_thetaDes));
	_droneController->_phiDes = min(_maxAngle, max(-_maxAngle, _droneController->_phiDes));
	_droneController->_psiDotDes = min(_maxPsiDot, max(-_maxPsiDot, _droneController->_psiDotDes));
	_droneController->_zDotDes = min(_maxZDot, max(-_maxZDot, _droneController->_zDotDes));

	//translate to be between -1 and 1 based on percentage of maximum
	_droneController->_thetaDes = _droneController->_thetaDes/_maxAngle;
	_droneController->_phiDes = _droneController->_phiDes/_maxAngle;
	_droneController->_psiDotDes = _droneController->_psiDotDes/_maxPsiDot;
	_droneController->_zDotDes = _droneController->_zDotDes/_maxZDot;

}


#endif
