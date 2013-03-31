#ifndef _CONTROLLER_CPP_
#define _CONTROLLER_CPP_
/*
 Controller.cpp
 Data type for the drone
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */

#include <iostream>

/*
 *QUADCOPTER_CONTROL INCLUDES
 */
#include "Controller.h"

/*Controller constructor (with default values) */
Controller::Controller() {
	_kpX = 0.00059;
	_kdX = 0.0025;
	_kiX = 0;
	_kpY = 0.00015;
	_kdY = 1.55e-6;
	_kiY = 0;
	_kpA = 0.19;
	_kdA = 0.0056;
	_kiA = 0;
	_kpT = 1.16;
	_kdT = 0;
	_kiT = 0;

	/*_eX = 0;
	_eY = 0;
	_eA = 0;
	_eT = 0;*/
	
	_totalXErr = 0;
	_totalYErr = 0;
	_totalAErr = 0;
	_totalTErr = 0;

	_lastXErr = 0;
	_lastYErr = 0;
	_lastAErr = 0;
	_lastTErr = 0;

	_thetaDes = 0;
	_phiDes = 0;
	_psiDotDes = 0;
	_zDotDes = 0;

}

/*Controller constructor (with specific values)*/
Controller::Controller(double kpX, double kiX, double kdX, double kpY, double kiY, double kdY, double kpA, double kiA, double kdA, double kpT, double kiT, double kdT) {
	_kpX = kpX;
	_kdX = kdX;
	_kiX = kiX;
	_kpY = kpY;
	_kdY = kdY;
	_kiY = kiY;
	_kpA = kpA;
	_kdA = kdA;
	_kiA = kiA;
	_kpT = kpT;
	_kdT = kdT;
	_kiT = kiT;

	/*_eX = 0;
	_eY = 0;
	_eA = 0;
	_eT = 0;*/
	
	_totalXErr = 0;
	_totalYErr = 0;
	_totalAErr = 0;
	_totalTErr = 0;

	_lastXErr = 0;
	_lastYErr = 0;
	_lastAErr = 0;
	_lastTErr = 0;

	_thetaDes = 0;
	_phiDes = 0;
	_psiDotDes = 0;
	_zDotDes = 0;
}

/* Controller destructor*/
Controller::~Controller(){

};

/*Take difference between two angles while keeping their difference between -180 and 180degees*/
double Controller::angleDiff(double a, double b) {
	double diff = a-b;

	while (diff > 180) {
		diff = diff-360;
	}
	while (diff < -180) {
		diff = diff+360;
	}

	return diff;
}

/*Reset control values*/
void Controller::resetController() {
	_totalXErr = 0;
	_totalYErr = 0;
	_totalAErr = 0;
	_totalTErr = 0;

	_lastXErr = 0;
	_lastYErr = 0;
	_lastAErr = 0;
	_lastTErr = 0;

	_thetaDes = 0;
	_phiDes = 0;
	_psiDotDes = 0;
	_zDotDes = 0;
}


void Controller::printState() {
	std::cout << "Xgains= " << _kpX << ", " << _kdX << ", " << _kiX << "; Ygains= " << _kpY << ", " << _kdY << ", " << _kiY << "; Agains= " << _kpA << ", " << _kdA << ", " << _kiA << "; Tgains= " << _kpT << ", " << _kdT << ", " << _kiT << std::endl;
	std::cout << "TotalErrs= " << _totalXErr << ", " << _totalYErr << ", " << _totalAErr << ", " << _totalTErr << "; LastErrs= " << _lastXErr << ", " << _lastYErr << ", " << _lastAErr << ", " << _lastTErr << std::endl ;
	std::cout << "DesiredVals= " << _thetaDes << ", " << _phiDes << ", " << _psiDotDes << ", " << _zDotDes << "\n" << std::endl;

}


#endif
