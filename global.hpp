/*** Glogbal variables ***/

#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#define PI 3.141592653589
#define beta 9

extern void global_init();

extern int LX, LY, NR_iterations, nr_samples, dt_save;   
extern double kin_visc, tau,inv_tau;;	  //kinematic viscosity
extern double Force[2];  // force
extern double Upper_Wall_velocity[2]; // velocity of the walls
extern double Lower_Wall_velocity[2];

extern double WLIST[9];
extern double cX[9];
extern double cY[9];
extern int INVLIST[9];

// extern double ***f, ***f_temp, **rho, **u_x, **du_x, **u_y, **du_y, **sigma,*f_eq, *f_eq_new;
// extern int **obstacle;

// extern char dens[200], velX[200], velY[200], shearstress[200];;
// FILE  *densityFile, *veloFile , *shearFile;

#endif
