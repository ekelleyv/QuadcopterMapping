#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_
/*
 functions.h
 Common functions
 Sarah Tang and Edward Francis Kelley V
 Senior thesis, 2012-2013
 */

/* 
 * C++ INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <errno.h>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

using namespace std;

void error(const char *msg);
double angleDiffDeg(double a, double b);
void help(int argc, char** argv);
int processCmdLine(int argc, char** argv, double* maxAngle, double* maxZDot, double* maxPsiDot, int* updateType);
int tasksLoadFile(const char* fname, std::vector<double>* xDes, std::vector<double>* yDes, std::vector<double>* aDes, std::vector<double>* tDes, std::vector<double>* hoverTime, int* numWaypoints);

#endif _FUNCTIONS_CPP
