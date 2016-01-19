/*** initialization of global variables ***/
/*
	Ypsilon 
*/

int LX, LY, NR_iterations, nr_samples, dt_save;   
double kin_visc, tau,inv_tau;	  //kinematic viscosity
double Force[2];  // force
double Upper_Wall_velocity[2]; // velocity of the walls
double Lower_Wall_velocity[2];

double WLIST[] = {4.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/36.0, 1.0/36.0, 1.0/36.0, 1.0/36.0};
double cX[] = {0.0, 1.0, 0.0, -1.0,  0.0, 1.0, -1.0, -1.0,  1.0};
double cY[] = {0.0, 0.0, 1.0,  0.0, -1.0, 1.0,  1.0, -1.0, -1.0};
int INVLIST[] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

void global_init(){
	// int LX, LY, NR_iterations, nr_samples, dt_save;   
	// double kin_visc, tau,inv_tau;	  //kinematic viscosity
	// double Force[2];  // force
	// double Upper_Wall_velocity[2]; // velocity of the walls
	// double Lower_Wall_velocity[2];

//	WLIST[] = {4.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/36.0, 1.0/36.0, 1.0/36.0, 1.0/36.0};
//	cX[] = {0.0, 1.0, 0.0, -1.0,  0.0, 1.0, -1.0, -1.0,  1.0};
//	cY[] = {0.0, 0.0, 1.0,  0.0, -1.0, 1.0,  1.0, -1.0, -1.0};
//	INVLIST[] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

//	double ***f, ***f_temp, **rho, **u_x, **du_x, **u_y, **du_y, **sigma,*f_eq, *f_eq_new;
//	int **obstacle;

//	char dens[200], velX[200], velY[200], shearstress[200];;
//	FILE  *densityFile, *veloFile , *shearFile;
	
}
