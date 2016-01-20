/*** Tpoint   one point in the lattice ***/

#ifndef TPOINT_HPP
#define TPOINT_HPP

#include <iostream>
#include <math.h>
#include <cassert>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "global.hpp"

using namespace std;

class Tpoint{
public:
	double u_x;
	double u_y;
	double rho;

	// double cX[beta]; // they are global
	// double cY[beta];
	double f[beta];
	double f_temp[beta];
	double f_eq[beta]; // can be global
	double f_eq_new[beta]; // can be global
	double du_x;
	double du_y;

	//double Force[2];
	Tpoint *neighbour[beta]; // pointer in direction (cX[i],cY[i])
	bool obstacle; // if is this point obstacle

	void computeEquilibriumDistribution();
	void computeNewEquilibriumDistribution();
	void computeDensityandVelocity();
	void propagation();
	void collision();
};


#endif
