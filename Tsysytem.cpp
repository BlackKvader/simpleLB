/*** main system class ***/

#include <sstream>
#include <string>
#include <cstdlib>

#include "Tpoint.hpp"
#include "global.hpp"
#include "Tsystem.hpp"

// helping functions. Just here
int PeriodicLX(int num);
int PeriodicLY(int num);

#define forAll(x) {\
	for (int polx=0;polx<LX;polx++){\
		for (int poly=0;poly<LY;poly++){\
			point[polx][poly].x;\
		}\
	}\
}

void Tsystem::run(){
	// loop
	write(100); // write output

	for (int i=0;i<20;i++){
		for (int x=0;x<LX;x++){
			for (int y=0;y<LY;y++){
				//point[x][y].computeDensityandVelocity();

				point[x][y].collision();

				point[x][y].propagation();
			}
		}

		write(i); // write output
	}
}

int PeriodicLX(int num){
	if (num<0){
		num=LX-1;
	} else {
		if (num>LX-2)
			num=0;
	}
	return num;
}
int PeriodicLY(int num){
	if (num<0){
		num=LY-1;
	} else {
		if (num>LY-2)
			num=0;
	}
	return num;
}

void Tsystem::init(){
	system("rm out");
	system("mkdir out");

	// set starting variables // later load from file ...
	LX = 41;
	LY = 1;
	// OK. I give up. But it was working for while ...

	// Tpoint size forced in class 

	kin_visc=0.16;
	tau = 3.0*kin_visc + 0.5; // initialized in global_init() as global variable
	// inv_tau= ... // never used in original code
	
	// make the system. Argh...
	// point = new Tpoint*[LX];
	// for (int i=0; i<LY; i++){
	// 	point[i]=new Tpoint[LY];
	// }

	// debuging
	// double z;
	// z = point[0][0].u_x;
	// cout << PeriodicLY(-1) << "  " << PeriodicLY(0) << "  " << PeriodicLY(1) << "  " << z << endl; 

	// set the neighbours for points
	for (int x=0;x<LX;x++){
		for (int y=0;y<LY;y++){
			for (int k=0;k<beta;k++){
				// 	in Tpoint => Tpoint *neighbour[beta]; // pointer in direction (cX[i],cY[i])
				//	double cX[] = {0.0, 1.0, 0.0, -1.0,  0.0, 1.0, -1.0, -1.0,  1.0};
				//	double cY[] = {0.0, 0.0, 1.0,  0.0, -1.0, 1.0,  1.0, -1.0, -1.0};
				point[x][y].neighbour[k]=&point[PeriodicLX(x+cX[k])][PeriodicLY(y+cY[k])]; // sehr schon

				// for "debug"
				point[x][y].u_x=x;
				point[x][y].u_y=y;
			}
		}
	}

	check();

	// init starting conditions
	for (int x=0;x<LX;x++){
		for (int y=0;y<LY;y++){
			point[x][y].rho = 1;
			point[x][y].u_x = 0;
			point[x][y].u_y = 0;
		}
	}

 	//set f to f_eq for initialization
	cout<< "setting f to f_eq for intialization "<<endl;
	for(int i=0;i<LX;i++){
		for(int j=0; j<LY;j++){
			point[i][j].computeEquilibriumDistribution();

	    	for (int k=0; k<beta ; k++){             
	        	point[i][j].f[k]    = 	point[i][j].f_eq[k];
	        	point[i][j].f_temp[k] = point[i][j].f_eq[k];
	    	}
		}
  	}

  	for (int k=0;k<beta;k++){
  		point[20][0].f[k]=k*2;
  		point[20][0].f_temp[k]=k*2;
  	}
  	

	cout<< "setting f to f_eq for intialization done. "<<endl;

	// obstacles
	forAll(obstacle=0);
	point[0][0].obstacle=1;
	point[LX-1][0].obstacle=1;

}

void Tsystem::check(){
	for (int k=0;k<beta;k++){
		cout << "point 20 0  neighbour " << k;
		cout << "  x: " << point[20][0].neighbour[k]->u_x << " cX " << cX[k];
		cout << "  y: " << point[20][0].neighbour[k]->u_y << " cY " << cY[k] << endl;
	}
	// for (int k=0;k<beta;k++){
	// 	cout << " cX " << cX[k] << " inv cX " << cX[INVLIST[k]] << "  ";
	// 	cout << " cY " << cY[k] << " inv cY " << cY[INVLIST[k]] << "  " << endl;
	// }
}

void Tsystem::close(){
	for (int i=0; i<LX; i++)
    	delete [] point[i];
	delete [] point;
}

void Tsystem::write(int num){
	stringstream ss;
	string s;
	ss << "out/out" << num << ".dat";
	s = ss.str();
	ofstream outfile(s.c_str());
	outfile.setf( std::ios::fixed, std:: ios::floatfield);
	outfile.precision(10);

	outfile << "f field" << endl;
	for (int x=0;x<LX;x++){
		outfile << x << "\t";
		for (int k=0;k<beta;k++){
			outfile << point[x][0].f[k] << "\t";
		}
		outfile << endl;
	}

	outfile << "f_temp field" << endl;
	for (int x=0;x<LX;x++){
		outfile << x << "\t";
		for (int k=0;k<beta;k++){
			outfile << point[x][0].f_temp[k] << "\t";
		}
		outfile << endl;
	}
}
