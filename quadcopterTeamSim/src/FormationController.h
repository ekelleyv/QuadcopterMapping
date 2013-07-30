#ifndef _FORMATIONCONTROLLER_H
#define _FORMATIONCONTROLLER_H
/*
 FormationController.h
 Data type for drone controller
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */


/*
 *QUADCOPTER_CONTROL INCLUDES
 */

class FormationController {

public: 
	//CONSTRUCTORS
	FormationController();
	FormationController(double ka1, double ka2, double kc1, double kc2);

	//DESTRUCTOR
	~FormationController();

	//PUBLIC FUNCTIONS
	double angleDiff(double a, double b);
	double angleSum(double a, double b);
	void resetController();
	void printState();

	//controller gains
	double _ka1;
	double _ka2;
	double _kc1;
	double _kc2;

	int _controlType; //0 for l-gamma, 1 for l-l
	int _leader1;
	int _leader2; //could be inf if none

	double _l1des;
	double _l2des;
	double _gamma1des;

	double _xl1;
	double _yl1;
	double _psil1;
	double _xl2;
	double _yl2;
	double _psil2;

	//desired values
	double _thetaDes;
	double _phiDes;
	double _psiDotDes;
	double _zDotDes;

};



#endif
