#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_
/*
 functions.cpp
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

void error(const char *msg);
double angleDiffDeg(double a, double b);
void help(int argc, char** argv);
int processCmdLine(int argc, char** argv, double* maxAngle, double* maxZDot, double* maxPsiDot);

#endif _FUNCTIONS_CPP