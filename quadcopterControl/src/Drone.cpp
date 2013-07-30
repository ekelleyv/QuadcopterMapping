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
Drone::Drone(double initialx, double initialy, double initialalt, int updateType) {
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

	_yawOffset = 0;

	_xn = 0;
	_yn = 0;
	_altn = 0;
	_un = 0;
	_vn = 0;
	_wn = 0;
	_udotn = 0;
	_vdotn = 0;
	_wdotn = 0;
	_thetan = 0;
	_phin = 0;
	_psin = 0;

	_xt = 0;
	_yt = 0;
	_altt = 0;
	_ut = 0;
	_vt = 0;
	_wt = 0;
	_thetat = 0;
	_phit = 0;
	_psit = 0;

	_xpf = 0;
	_ypf = 0;
	_zpf = 0;

	_maxAngle = 12;
	_maxZDot = 700;
	_maxPsiDot = 100;

	lastStateTime = ros::Time::now();
	lastControlTime = ros::Time::now();
	lastUpdateTime = ros::Time::now();
	lastTumTime = ros::Time::now();
	lastPFTime = ros::Time::now();

	_droneController = new Controller();
	_updateType = updateType;
}


Drone::Drone(double initialx, double initialy, double initialalt, double kpX, double kiX, double kdX, double kpY, double kiY, double kdY, double kpA, double kiA, double kdA, double kpT, double kiT, double kdT, int updateType) {

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

	_yawOffset = 0;

	_xn = initialx;
	_yn = initialy;
	_altn = initialalt;
	_un = 0;
	_vn = 0;
	_wn = 0;
	_udotn = 0;
	_vdotn = 0;
	_wdotn = 0;
	_thetan = 0;
	_phin = 0;
	_psin = 0;

	_xt = initialx;
	_yt = initialy;
	_altt = initialalt;
	_ut = 0;
	_vt = 0;
	_wt = 0;
	_thetat = 0;
	_phit = 0;
	_psit = 0;

	_xpf = 0;
	_ypf = 0;
	_zpf = 0;

	lastStateTime = ros::Time::now();
	lastControlTime = ros::Time::now();
	lastUpdateTime = ros::Time::now();
	lastTumTime = ros::Time::now();
	lastPFTime = ros::Time::now();

	_droneController = new Controller(kpX, kiX, kdX, kpY, kiY, kdY, kpA, kiA, kdA, kpT, kiT, kdT);
	_updateType = updateType;

}

/* Drone destructor*/
Drone::~Drone(){
	_droneController->~Controller();
};

/*Update the state of the drone*/
void Drone::updateNavState(const ardrone_autonomy::Navdata::ConstPtr& msg) {
	//msg->state - use drone state somehow?
	//invert signs for it to match our convention

	_un = msg->vx;
	_vn = msg->vy;
	_wn = msg->vz;
	_udotn = msg->ax;
	_vdotn = msg->ay;
	_wdotn = msg->az;
	_phin = msg->rotX;
	_thetan =  msg->rotY;
	_psin = _droneController->angleSum(msg->rotZ, _yawOffset); //msg->rotZ;
	_altn = msg->altd;

	//use dead reckoning for position estimates
	//rotate global position into the drone's frame
	double dt = (msg->header.stamp - lastStateTime).toSec();
	lastStateTime = msg->header.stamp;
	_xn = _xn + ((- _un*cos(_psin*M_PI/180) + _vn*sin(_psin*M_PI/180))*dt);
	_yn = _yn + ((- _un*sin(_psin*M_PI/180) - _vn*cos(_psin*M_PI/180))*dt);


}

/*Update the state of the drone*/
void Drone::updateTumState(const tum_ardrone::filter_state::ConstPtr& msg) {
	//msg->state - use drone state somehow?
	//invert cordinate frames nad signs to match our convention

	_xt = msg->y*1000;
	_yt = -msg->x*1000;
	_altt = msg->z*1000;
	_ut = msg->dy*1000;
	_vt = -msg->dx*1000;
	_wt = msg->dz;
	_phit = msg->roll;
	_thetat =  -msg->pitch;
	_psit = _droneController->angleDiff(msg->yaw, 180);

	lastTumTime = msg->header.stamp;

}

/*Update particle filter state*/
void Drone::updatePFState(const geometry_msgs::Pose::ConstPtr& msg) {

	_xpf = msg->position.x;
	_ypf = msg->position.y;
	_zpf = msg->position.z;
	_orient = msg->orientation;

	lastPFTime = ros::Time::now();

	std::cout << "updating " << _xpf << " " << _ypf << " " << _zpf << std::endl;
}


/* Update state*/
void Drone::updateState() {
	if (_updateType == 0) {
		_u = _un;
		_v = _vn;
		_w = _wn;
		_udot = _udotn;
		_vdot = _vdotn;
		_wdot = _wdotn;
		_phi = _phin;
		_theta =  _thetan;
		_psi = _psin;
		_alt = _altn;
		_x = _xn;
		_y = _yn;
		lastUpdateTime = lastStateTime;
	}

	else if (_updateType == 1) {
		_x = _xt;
		_y = _yt;
		_alt = _altt;
		_u = _ut;
		_v = _vt;
		_w = _wt;
		_phi = _phit;
		_theta = _thetat;
		_psi = _psit;
		lastUpdateTime = lastTumTime;
	}
	else if (_updateType == 2) {
		double dt = (lastTumTime - lastUpdateTime).toSec();
		lastUpdateTime = lastTumTime;
		_x = _x + _ut*dt;
		_y = _y + _vt*dt;
		_alt = _altn;
		_u = _ut;
		_v = _vt;
		_w = _wt;
		_phi = _phit;
		_theta = _thetat;
		_psi = _psin;
	}
	else if (_updateType == 3) {
		double dt = (lastStateTime - lastUpdateTime).toSec();
		lastUpdateTime = lastStateTime;
		_x = _x + ((- _un*cos(_psin*M_PI/180) + _vn*sin(_psin*M_PI/180))*dt);
		_y = _y + ((- _un*sin(_psin*M_PI/180) - _vn*cos(_psin*M_PI/180))*dt);

		_u = _un;
		_v = _vn;
		_w = _wn;
		_udot = _udotn;
		_vdot = _vdotn;
		_wdot = _wdotn;
		_phi = _phin;
		_theta =  _thetan;
		_psi = _psin;
		_alt = _altn;
	}
	else if (_updateType == 4) {
		_x = _xpf;
		_y = _ypf;	
		_alt = _zpf;
		//_alt = _altn;
		_psi = _droneController->angleDiff((tf::getYaw(_orient)*180/M_PI), 180);
		lastUpdateTime = lastPFTime;
	}

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
void Drone::calculateControl(double desX, double desY, double desA, double desT) {
	double dt = (ros::Time::now() - lastControlTime).toSec();
	lastControlTime = ros::Time::now();

	double eXGlobal = desX-_x;
	double eYGlobal = desY-_y;
	double eX = (- eXGlobal*cos(_psi*M_PI/180)) - (eYGlobal*sin(_psi*M_PI/180));
	double eY = (eXGlobal*sin(_psi*M_PI/180)) - (eYGlobal*cos(_psi*M_PI/180));
	double eA = desA-_alt;
	double eT = _droneController->angleDiff(desT, _psi);

	//update integral error
	/*_droneController->_totalXErr = _droneController->_totalXErr + eX;
	_droneController->_totalYErr = _droneController->_totalYErr + eY;
	_droneController->_totalAErr = _droneController->_totalAErr + eA;
	_droneController->_totalTErr = _droneController->_totalTErr + eT;*/
	_droneController->_totalXErr = 0;
	_droneController->_totalYErr = 0;
	_droneController->_totalAErr = 0;
	_droneController->_totalTErr = 0;

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


void Drone::calculateXControl(double desX, double desY) {
	double dt = (ros::Time::now() - lastControlTime).toSec();
	lastControlTime = ros::Time::now();

	double eXGlobal = desX-_x;
	double eYGlobal = desY-_y;
	double eX = (-eXGlobal*cos(_psi*M_PI/180)) - (eYGlobal*sin(_psi*M_PI/180));

	//update integral error
	_droneController->_totalXErr = _droneController->_totalXErr + eX;

	//calculate PID signal
	_droneController->_thetaDes = (_droneController->_kpX*eX) + (_droneController->_kiX*_droneController->_totalXErr) + (_droneController->_kdX*_u);

	//update last error
	_droneController->_lastXErr = eX;
}



void Drone::calculateYControl(double desX, double desY) {
	double dt = (ros::Time::now() - lastControlTime).toSec();
	lastControlTime = ros::Time::now();

	double eXGlobal = desX-_x;
	double eYGlobal = desY-_y;
	double eY = (eXGlobal*sin(_psi*M_PI/180)) - (eYGlobal*cos(_psi*M_PI/180));

	//update integral error
	_droneController->_totalYErr = _droneController->_totalYErr + eY;

	//calculate PID signal
	_droneController->_phiDes = (_droneController->_kpY*eY) + (_droneController->_kiY*_droneController->_totalYErr) + (_droneController->_kdY*_v);

	//update last error
	_droneController->_lastYErr = eY;

}



void Drone::calculateAControl(double desA) {
	double dt = (ros::Time::now() - lastControlTime).toSec();
	lastControlTime = ros::Time::now();

	double eA = desA-_alt;

	//update integral error
	_droneController->_totalAErr = _droneController->_totalAErr + eA;

	//calculate PID signal
	_droneController->_zDotDes = (_droneController->_kpA*eA) + (_droneController->_kiA*_droneController->_totalAErr) + (_droneController->_kdA*(_droneController->_lastAErr - eA)/dt);
	
	//update last error
	_droneController->_lastAErr = eA;
}



void Drone::calculateYawControl(double desT) {
	double dt = (ros::Time::now() - lastControlTime).toSec();
	lastControlTime = ros::Time::now();

	double eT = _droneController->angleDiff(desT, _psi);

	//update integral error
	_droneController->_totalTErr = _droneController->_totalTErr + eT;

	//calculate PID signal
	_droneController->_psiDotDes = (_droneController->_kpT*eT) + (_droneController->_kiT*_droneController->_totalTErr) + (_droneController->_kdT*(_droneController->_lastTErr - eT)/dt);
	
	//update last error
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
