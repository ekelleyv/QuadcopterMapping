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
Drone::Drone(double initialx, double initialy, double initialalt, int updateType, int droneType, int index) {
	_droneType = droneType;	
	_index = index;

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
	_psi = -180;

	_yawOffset = 0;

	//additional variables for simulations
	_psidot = 0;
	_thetadot = 0;
	_phidot = 0;
	_psiddot = 0;

	_thetades = 0;
	_phides = 0;
	_psidotdes = 0;
	_zdotdes = 0;

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
	_psin = -180;

	_xt = initialx;
	_yt = initialy;
	_altt = initialalt;
	_ut = 0;
	_vt = 0;
	_wt = 0;
	_thetat = 0;
	_phit = 0;
	_psit = -180;

	_xpf = initialx;
	_ypf = initialy;
	_zpf = initialalt;

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

	_droneFController = new FormationController();
}


Drone::Drone(double initialx, double initialy, double initialalt, int updateType, int droneType, int index, double kpX, double kiX, double kdX, double kpY, double kiY, double kdY, double kpA, double kiA, double kdA, double kpT, double kiT, double kdT, double ka1, double ka2, double kc1, double kc2) {
	_droneType = droneType;
	_index = index;

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

	_xpf = initialx;
	_ypf = initialy;
	_zpf = initialalt;

	lastStateTime = ros::Time::now();
	lastControlTime = ros::Time::now();
	lastUpdateTime = ros::Time::now();
	lastTumTime = ros::Time::now();
	lastPFTime = ros::Time::now();

	_droneController = new Controller(kpX, kiX, kdX, kpY, kiY, kdY, kpA, kiA, kdA, kpT, kiT, kdT);
	_updateType = updateType;

	_droneFController = new FormationController(ka1, ka2, kc1, kc2);

}

/* Drone destructor*/
Drone::~Drone(){
	_droneController->~Controller();
	_droneFController->~FormationController();
};

/*Simulate the Drone moving forward*/
void Drone::simulateDrone() {
	double dt = (ros::Time::now() - lastControlTime).toSec();
	lastControlTime = ros::Time::now();

	double theta_new = 0.9975*_theta + 0.004744*_thetadot+4.783e-6*_thetades;
	double thetadot_new = -0.0006492*_theta + _thetadot+0.0017*_thetades;
	double u_new = 3.917*_theta + 0.04574*_thetadot + 0.9822*_u + 0.006981*_udot+8.152e-5*_thetades;
	double udot_new = -15.29*_theta - 0.1695*_thetadot - 0.02204*_u + 0.8575*_udot-0.000282*_thetades;

	double phi_new = 0.9975*_phi + 0.004744*_phidot+5.982e-6*_phides;
	double phidot_new = -0.0006991*_phi + _phidot + 0.0021*_phides;
	double v_new = -4.627*_phi - 0.05476*_phidot + 0.9847*_v + 0.00709*_vdot - 0.0001254*_phides;
	double vdot_new = 3.762*_phi + 0.04477*_phidot - 0.004185*_v + 0.9847*_vdot + 0.0001055*_phides;

	double psidot_new = 0.9562*_psidot + 0.00489*_psiddot + 0.0003824*_psidotdes;
	double psiddot_new = -0.1704*_psidot + 0.9996*_psiddot + 0.1552*_psidotdes;

	double w_new = 0.987*_w + 0.004968*_wdot + 0.001002*_zdotdes;
	double wdot_new = -0.4323*_w + 0.9989*_wdot + 0.4024*_zdotdes;

	_theta = _droneController->angleSum(theta_new, 0);
	_thetadot = _droneController->angleSum(thetadot_new, 0);
	_u = u_new;
	_udot = udot_new;
	_phi = _droneController->angleSum(phi_new, 0);
	_phidot = _droneController->angleSum(phidot_new, 0);
	_v = v_new;
	_vdot = vdot_new;

	_psidot = _droneController->angleSum(psidot_new, 0);
	_psiddot = _droneController->angleSum(psiddot_new, 0);

	_w = w_new;
	_wdot = wdot_new;

	_psi = _droneController->angleSum(_psi, _psidot*dt);
	_alt = _alt + _w*dt;

	_x = _x + ((- _u*cos(_psi*M_PI/180) + _v*sin(_psi*M_PI/180))*dt);
	_y = _y + ((- _u*sin(_psi*M_PI/180) - _v*cos(_psi*M_PI/180))*dt);
}

/*Update drone commands*/
void Drone::simulateCommands(double thetades, double phides, double psidotdes, double zdotdes) {
	_thetades = thetades;
	_phides = phides;
	_psidotdes = psidotdes;
	_zdotdes = zdotdes;
}

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
	_psin = _droneController->angleSum(msg->rotZ, _yawOffset);
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
		_alt = _altn;
		_u = _ut;
		_v = _vt;
		_w = _wt;
		_phi = _phit;
		_theta = _thetat;
		_psi = _psin;
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

	std::cout << "\nType= " << _droneType << "; Position= " << _x << ", " << _y << ", " << _alt << "; LinearSpeed= " << _u << ", " << _v << ", " << _w << "; LinearAcceleration= " << _udot << ", " << _vdot << ", " << _wdot << "; Angles= " << _theta << ", " << _phi << ", " << _psi << std::endl;
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

/*Calculate, from drones in view and their distnaces, what control law to use*/
void Drone::simulateControlGraph(std::vector< std::vector<double> > formation, std::vector< std::vector<double> > formationGamma, int thisIndex, int numRobots, std::vector<Drone*> allRobots) {

	//global information
		//hardcore for now
		if (thisIndex == 1) {
		_droneFController->_controlType = 0;
		_droneFController->_leader1 = 0;
		_droneFController->_leader2 = std::numeric_limits<int>::max(); 

		_droneFController->_l1des = formation[_droneFController->_leader1][thisIndex];
		_droneFController->_gamma1des = formationGamma[_droneFController->_leader1][thisIndex];
		_droneFController->_l2des = std::numeric_limits<int>::max();

		_droneFController->_xl1 = allRobots[_droneFController->_leader1]->_x; 
		_droneFController->_yl1 = allRobots[_droneFController->_leader1]->_y; 
		_droneFController->_psil1 = allRobots[_droneFController->_leader1]->_psi; 
		_droneFController->_xl2 = std::numeric_limits<int>::max();
		_droneFController->_yl2 = std::numeric_limits<int>::max();
		_droneFController->_psil2 = std::numeric_limits<int>::max();	
		}
		else if (thisIndex >= 2) {
		
		_droneFController->_controlType = 1;
		_droneFController->_leader1 = 0;
		_droneFController->_leader2 = 1; 

		_droneFController->_l1des = formation[_droneFController->_leader1][thisIndex];
		_droneFController->_gamma1des = formationGamma[_droneFController->_leader1][thisIndex];
		_droneFController->_l2des = formation[_droneFController->_leader2][thisIndex];

		_droneFController->_xl1 = allRobots[_droneFController->_leader1]->_x; 
		_droneFController->_yl1 = allRobots[_droneFController->_leader1]->_y; 
		_droneFController->_psil1 = allRobots[_droneFController->_leader1]->_psi; 
		_droneFController->_xl2 = allRobots[_droneFController->_leader2]->_x; 
		_droneFController->_yl2 = allRobots[_droneFController->_leader2]->_y; 
		_droneFController->_psil2 = allRobots[_droneFController->_leader2]->_psi; 
		}


	//limited sensing for robots
	

	/*double minL; 
	double minGamma; 

	double minDist = std::numeric_limits<int>::max();
	int minIndex1 = -1;
	int minIndex2 = -1; */
}

/*Calcualte LGammaControl*/
void Drone::calculateFormationControl() {

	//if l-gamma control
	if (_droneFController->_controlType == 0) {

	double alpha = _droneFController->angleSum(180, -_droneFController->_gamma1des);
	double b1des = -_droneFController->_l1des*cos(alpha*M_PI/180);
	double b2des = _droneFController->_l1des*sin(alpha*M_PI/180);

std::cout << "control type 0, leaderstate= " <<  _droneFController->_xl1 << " " << _droneFController->_yl1 << " " << _droneFController->_psil1 << " desiredlgamma= " << _droneFController->_l1des << " " << _droneFController->_gamma1des << " bvals= " << b1des << " " << b2des << std::endl;

	double da1 = -(_x-_droneFController->_xl1)*cos(_psi*M_PI/180) - (_y-_droneFController->_yl1)*sin(_psi*M_PI/180) - cos((_droneFController->angleSum(-_psi, _droneFController->_psil1))*M_PI/180)*b1des + sin((_droneFController->angleSum(-_psi, _droneFController->_psil1))*M_PI/180)*b2des;
	double da2 = (_x-_droneFController->_xl1)*sin(_psi*M_PI/180) - (_y-_droneFController->_yl1)*cos(_psi*M_PI/180) - sin((_droneFController->angleSum(-_psi, _droneFController->_psil1))*M_PI/180)*b1des -cos((_droneFController->angleSum(-_psi, _droneFController->_psil1))*M_PI/180)*b2des;

	std::cout << "das= " << da1 << " " << da2 << std::endl;


	_droneFController->_thetaDes = _droneFController->_ka1*da1;
	_droneFController->_phiDes = _droneFController->_ka2*da2;
	_droneFController->_zDotDes = 0;
	_droneFController->_psiDotDes = 0;
	}
	else if (_droneFController->_controlType == 1) {	
		double x1 = _droneFController->_xl1;
		double x2 = _droneFController->_xl2;
		double psi1 = _droneFController->_psil1;
		double y1 = _droneFController->_yl1;
		double y2 = _droneFController->_yl2;

		double d = sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
		double a = (_droneFController->_l1des*_droneFController->_l1des - _droneFController->_l2des*_droneFController->_l2des + d*d)/2/d;
		double h = sqrt(_droneFController->_l1des*_droneFController->_l1des-a*a);

		double xtemp = x1 + a*(x2-x1)/d;
		double ytemp = y1 + a*(y2-y1)/d;

		double xdes1 = xtemp + h*(y2-y1)/d;
		double ydes1 = ytemp - h*(x2-x1)/d;
		double xdes2 = xtemp - h*(y2-y1)/d;
		double ydes2 = ytemp + h*(x2-x1)/d;

		double atest1 = -cos(psi1*M_PI/180)*xdes1 - sin(psi1*M_PI/180)*ydes1;
		double atest2 = -cos(psi1*M_PI/180)*xdes2 - sin(psi1*M_PI/180)*ydes2;
		double xdes;
		double ydes;
		if (atest1<atest2) {
			xdes = xdes1;
			ydes = ydes1;
		}
		else {
			xdes = xdes2;
			ydes = ydes2;
		}

		
		//if solution exists, l-l control
		if ( !isnan(xdes) && !isnan(ydes) ) {
		double dc1 = -cos(_psi*M_PI/180)*(_x-xdes) - sin(_psi*M_PI/180)*(_y-ydes);
		double dc2 = sin(_psi*M_PI/180)*(_x-xdes) - cos(_psi*M_PI/180)*(_y-ydes);

		_droneFController->_thetaDes = _droneFController->_kc1*dc1;
		_droneFController->_phiDes = _droneFController->_kc2*dc2;
		_droneFController->_zDotDes = 0;
		_droneFController->_psiDotDes = 0;
		}

std::cout << "control type 1, leaderstate= " <<  _droneFController->_xl1 << " " << _droneFController->_yl1 << " " << _droneFController->_psil1 << " leader2state= " << _droneFController->_xl2 << " " << _droneFController->_yl2 << " " << _droneFController->_psil2 << " desiredl= " << _droneFController->_l1des << " " << _droneFController->_l2des << " desiredPoints= " << xdes << " " << ydes << std::endl;


/*		else { //revert to l-gamma control

		double alpha = _droneFController->angleSum(180, -_droneFController->_gamma1des);
		double b1des = -_droneFController->_l1des*cos(alpha*M_PI/180);
		double b2des = _droneFController->_l1des*sin(alpha*M_PI/180);

		double da1 = -(_x-_droneFController->_xl1)*cos(_psi*M_PI/180) - (_y-_droneFController->_yl1)*sin(_psi*M_PI/180) - cos((_droneFController->angleSum(-_psi, _droneFController->_psil1))*M_PI/180)*b1des + sin((_droneFController->angleSum(-_psi, _droneFController->_psil1))*M_PI/180)*b2des;
		double da2 = (_x-_droneFController->_xl1)*sin(_psi*M_PI/180) - (_y-_droneFController->_yl1)*cos(_psi*M_PI/180) - sin((_droneFController->angleSum(-_psi, _droneFController->_psil1))*M_PI/180)*b1des -cos((_droneFController->angleSum(-_psi, _droneFController->_psil1))*M_PI/180)*b2des;
	
		_droneFController->_thetaDes = _droneFController->_ka1*da1;
		_droneFController->_phiDes = _droneFController->_ka2*da2;
		_droneFController->_zDotDes = 0;
		_droneFController->_psiDotDes = 0;

		}*/


	}
}


/*Transate control signals to be between -1 and 1 of maximum*/
void Drone::translateLFControl() {
	//limit to maximums
	_droneFController->_thetaDes = min(_maxAngle, max(-_maxAngle, _droneFController->_thetaDes));
	_droneFController->_phiDes = min(_maxAngle, max(-_maxAngle, _droneFController->_phiDes));
	_droneFController->_psiDotDes = min(_maxPsiDot, max(-_maxPsiDot, _droneFController->_psiDotDes));
	_droneFController->_zDotDes = min(_maxZDot, max(-_maxZDot, _droneFController->_zDotDes));

	//translate to be between -1 and 1 based on percentage of maximum
	_droneFController->_thetaDes = _droneFController->_thetaDes/_maxAngle;
	_droneFController->_phiDes = _droneFController->_phiDes/_maxAngle;
	_droneFController->_psiDotDes = _droneFController->_psiDotDes/_maxPsiDot;
	_droneFController->_zDotDes = _droneFController->_zDotDes/_maxZDot;

}


#endif
