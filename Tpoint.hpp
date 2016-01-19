/*** Tpoint   one point in the lattice ***/

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
	double du_x[beta];
	double du_y[beta];

	double *Force[2];
	Tpoint *neighbour[beta]; // pointer in direction (cX[i],cY[i])
	bool obstacle; // if is this point obstacle

	void computeEquilibriumDistribution();
	void computeNewEquilibriumDistribution();
	void computeDensityandVelocity();
};

void Tpoint::computeEquilibriumDistribution(){
  for (int k=0; k<beta ; k++)
    {
      f_eq[k] = WLIST[k]*rho*(\
      	1.0+\
      	3.0*(u_x*cX[k] + u_y*cY[k]) +\
      	(9.0/2.0)*(u_x*cX[k] + u_y*cY[k])*(u_x*cX[k] + u_y*cY[k]) -\
      	(3.0/2.0)*(u_x*u_x + u_y*u_y)\
      );
    };
}

//This code makes use of the so called exact difference method. Within this approach, the effect of force in the collision step is accounted for via a shift of the equilibrium distribution around u to a new distribution around u+du, where du=F dt /rhom (= F/rho since dt=1). One thus adds feq(u+du)-feq(u) to the collision operator. We, therefore, evaluate here feq(u+du) and save it in f_eq_new. See also the routine Collision().
void Tpoint::computeNewEquilibriumDistribution(){
  for (int k=0; k<beta ; k++)
    {
      f_eq_new[k] = WLIST[k]*rho*(\
      	1.0+ \
      	3.0*		((u_x+du_x)*cX[k] 		+ (u_y+du_y)*cY[k]) + \
      	(9.0/2.0)*	((u_x+du_x)*cX[k] 		+ (u_y+du_y)*cY[k])*((u_x+du_x)*cX[k] + (u_y+du_y)*cY[k])-\
      	(3.0/2.0)*	((u_x+du_x)*(u_x+du_x) 	+ (u_x+du_x)*(u_y+du_y) )\
      );
    }
}

void Tpoint::computeDensityandVelocity(){
	double sum = 0;
	double sumX = 0;
	double sumY = 0;

	for (int k=0; k<beta ; k++){
		sum = sum + f[k];
		sumX = sumX + f[k]*cX[k];
		sumY = sumY + f[k]*cY[k];
	}

	rho = sum;
	u_x = (sumX )/sum;
	u_y = (sumY)/sum;
	du_x= Force[0]/sum;
	du_y= Force[1]/sum;
}

void Tpoint::propagation(){
	for (int k=0;k<beta;k++){
		if (neighbour[k].obstacle==0){
			// No obstacle
			point_dir[k]->f[k]=f_temp[k];
		} else {
			/********************** CONTROL ok, should be good now *********************/
			// obstacle = bounce back
			f[INVLIST[k]]=f_temp[k]; // nice to use INVLIST
		}
	}
}

void Tpoint::Collision()
{
  if( obstacle[i][j] != 1) {
		//apply collision for non-solid nodes only
		computeEquilibriumDistribution();
		computeNewEquilibriumDistribution();
		//ComputeFLB (i,j);
		for (int k=0; k<beta ; k++){
			//In the line below, (f_eq_new[k]-f_eq[k]) accounts for the effect of force within the exact difference method.
		  f_temp[k] = f[k]*(1.0- 1.0/tau) + (1.0/tau)*f_eq[k] +  (f_eq_new[k]-f_eq[k]);
		}
  }
}
