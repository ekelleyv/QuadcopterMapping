#ifndef _FUNCTIONS_CPP_
#define _FUNCTIONS_CPP_
/*
 functions.cpp
 Common functions
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */

//Project includes
#include "functions.h"

/*Print error message*/
void error(const char *msg)
{
    if (errno != 0) { perror(msg);}
    else { fprintf(stderr,"%s", msg);}
    exit(1);
}

/*Difference between two angles, bounded between 180 and -180*/
double angleDiffDeg(double a, double b) {
	double diff = a-b;

	while (diff > 180) {
		diff = diff-360;
	}
	while (diff < -180) {
		diff = diff+360;
	}

	return diff;
}

void help(int argc, char** argv){
    fprintf(stderr,
            "\n"
            "Quadcopter Control Code\n"
            ); 
    
    fprintf(stderr,
            "\n"
            "USAGE: \n"
            "       -h for help\n"
            "       -x MAX_ANGLE\n"
	    " 		Set the maximum angle, default is 12 degrees\n"
            "       -a MAX_ALT\n"
            "           Set maximum altitude rate, maximum is 700mm/sec\n"
            "       -t MAX_YAW_RATE\n"
            "           Set the maximum yaw rate, default is 100 deg/sec\n"
	    " 	    -u UPDATE_TYPE \n"
            "   	Set the update type, 0 for navdata, 1 for tum_ardrone, 2 for tum_ardrone integrated, 3 for mix between tum_adrone and switch to navdata \n"
            );
    
    fprintf(stderr,"\n");
}

/*
 * Process the command line
 *
 * input in form [exectauble] 
 * -h: help
 * -x: max angle
 * -a: maximum altitude
 * -t: maximum yaw rate
 * -u: update type
 */
int processCmdLine(int argc, char** argv, double* maxAngle, double* maxZDot, double* maxPsiDot, int* updateType) {
    int c; 
    
    //process options
    while (1){
        c =  getopt (argc, argv, "ha:z:y:u:");
        if(c == -1) break;
        switch(c){
                //for help
            case 'h':
                help(argc,argv);
                exit(0);
                break;
            case 'x':
                *maxAngle = atoi(optarg);
                break;
            case 'a':
                *maxZDot = atoi(optarg);
                break;
            case 't':
                *maxPsiDot = atoi(optarg);
                break;
	    case 'u': 
		std:: cout << "switching update type.." << std::endl;
		*updateType = atoi(optarg);
            default:
                return -1;
        }
    }
    
    if(optind < argc){ std::cout << optind << " " << argc << " " << argv[1] << " " << argv[2] << std::endl; error("Unparsed commandline parameters\n");}
    
    return 0;
}

/*
 * Loads waypoints from the textfile waypoints.txt
 * Assumes waypoints.txt is in current folder.
 * 
 * file format: 
 * 	[Desired x],[Desired y],[Desired z],[Desired yaw]
 * 	...
 * 
 * NO SPACES BETWEEN COMMAS IN FILENAME
 */
int tasksLoadFile(const char* fname, std::vector<double>* xDes, std::vector<double>* yDes, std::vector<double>* aDes, std::vector<double>* tDes, std::vector<double>* hoverTime, int* numWaypoints){
    ifstream inFile (fname);
    string line;
    int linenum = 0;
    while (getline (inFile, line))
    {
        linenum++;
        //cout << "\nLine #" << linenum << ":" << endl;
        istringstream linestream(line);
        string item;
        int itemnum = 0;
        while (getline (linestream, item, ','))
        {
            itemnum++;
	    
	    switch(itemnum){
                //for help
            case 1:
                xDes->push_back(atof(item.c_str()));
                break;
            case 2:
                yDes->push_back(atof(item.c_str()));
                break;
            case 3:
                aDes->push_back(atof(item.c_str()));
                break;
            case 4:
                tDes->push_back(atof(item.c_str()));
                break;
	    case 5:
		hoverTime->push_back(atof(item.c_str()));
		break;
            default:
                error("Too many fields in waypoints.txt!");
            }
            cout << "Item #" << itemnum << ": " << item << endl;
        }
    }

    *numWaypoints = linenum;

    return 0;
}

#endif _FUNCTIONS_CPP    
