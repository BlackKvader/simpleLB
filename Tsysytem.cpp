/*** main system class ***/

#include "Tpoint.hpp"
#include "global.hpp"

// helping functions
int PeriodicLX(int num);
int PeriodicLY(int num);

class Tsystem{
public:
	Tpoint **point;

	// functions
	void read_parameter_file(const string parameter_filename); // probably need const *char ...
	void run();
	void init();
};

void Tsysytem::run(){
	// loop
	for (int i=0;i<20;i++){
		computeDensityandVelocity();

	  Collision();

	  Propagation();
	}
}

int PeriodicLX(int num){
	if (num<0)
		return LX;
	if (num>LX-1)
		return 0;
}
int PeriodicLY(int num){
	if (num<0)
		return LY;
	if (num>LY-1)
		return 0;
}

void Tsysytem::init(){
	// set starting variables // maybe load from file ...
	LX = 10;
	LY = 10;
	kin_visc=0.16;
	// tau= // initialized in global_init() as global variable
	// inv_tau= ... // never used in original code
	// make the system
	point = new Tpoint*[LX];
	for (int i=0; i<LY; i++){
		point[i]=new Tpoint[LY];
	}

	// set the neighbours for points
	for (int x=0;x<LX;x++){
		for (int y=0;y<LY;y++){
			for (int k=0;k<beta;k++){
				// 	in Tpoint => Tpoint *neighbour[beta]; // pointer in direction (cX[i],cY[i])
				//	double cX[] = {0.0, 1.0, 0.0, -1.0,  0.0, 1.0, -1.0, -1.0,  1.0};
				//	double cY[] = {0.0, 0.0, 1.0,  0.0, -1.0, 1.0,  1.0, -1.0, -1.0};
				point[x][y].neighbour[k]=&point[PeriodicLX(x+cX[k])][PeriodicLY(y+cY[k])]; // sehr schon
			}
		}
	}

	// init starting conditions
	for (int x=0;x<LX;x++){
		for (int y=0;y<LY;y++){
			point[x][y].rho = 1;
		}
	}
}
