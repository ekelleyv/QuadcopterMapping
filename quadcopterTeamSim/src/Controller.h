#ifndef _CONTROLLER_H
#define _CONTROLLER_H
/*
 Controller.h
 Data type for drone controller
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */


/*
 *QUADCOPTER_CONTROL INCLUDES
 */

class Controller {

public: 
	//CONSTRUCTORS
	Controller();
	Controller(double kpX, double kiX, double kdX, double kpY, double kiY, double kdY, double kpA, double kiA, double kdA, double kpT, double kiT, double kdT);

	//DESTRUCTOR
	~Controller();

	//PUBLIC FUNCTIONS
	double angleDiff(double a, double b);
	double angleSum(double a, double b);
	void resetController();
	void resetXController();
	void resetYController();
	void resetAController();
	void resetYawController();
	void printState();

	//controller gains
	double _kpX;
	double _kiX;
	double _kdX;
	double _kpY;
	double _kiY;
	double _kdY;
	double _kpA;
	double _kiA;
	double _kdA;
	double _kpT;
	double _kiT;
	double _kdT;

	//keep track of error values
	/*double _eX;
	double _eY;
	double _eA;
	double _eT;*/

	//keep track of error values - integral error
	double _totalXErr;
	double _totalYErr;
	double _totalAErr;
	double _totalTErr;

	//keep track of error values - derivative error
	double _lastXErr;
	double _lastYErr;
	double _lastAErr;
	double _lastTErr;

	//desired values
	double _thetaDes;
	double _phiDes;
	double _psiDotDes;
	double _zDotDes;


};



#endif
