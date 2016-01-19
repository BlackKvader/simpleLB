#include <iostream>
#include <math.h>
#include <cassert>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
using namespace std;
#define PI 3.141592653589

#define beta 9

int LX, LY, NR_iterations, nr_samples, dt_save;   
double kin_visc, tau,inv_tau;;	  //kinematic viscosity
double Force[2];  // force
double Upper_Wall_velocity[2]; // velocity of the walls
double Lower_Wall_velocity[2];

double WLIST[] = {4.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/9.0, 1.0/36.0, 1.0/36.0, 1.0/36.0, 1.0/36.0};
double cX[] = {0.0, 1.0, 0.0, -1.0,  0.0, 1.0, -1.0, -1.0,  1.0};
double cY[] = {0.0, 0.0, 1.0,  0.0, -1.0, 1.0,  1.0, -1.0, -1.0};
int INVLIST[] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

double ***f, ***f_temp, **rho, **u_x, **du_x, **u_y, **du_y, **sigma,*f_eq, *f_eq_new;
int **obstacle;

char dens[200], velX[200], velY[200], shearstress[200];
FILE  *densityFile, *veloFile , *shearFile;

void read_parameter_file(char inputname[])
{

  ifstream infile;

/*  Clasic input file

1 41
10000 10
0.16
1.0e-6 0.0
0.e-6 0.0
0.e-6 0.0

*/

  infile.open(inputname);
  if(!infile)
    {
      cout << "read_parameter_file: Could not open " << inputname << endl;
      cout << "  ==> Exitting to the system..." << endl;
      exit(13);
    }

  if (infile.is_open())
    {
      cout << "read_parameter_file: reading system parameters from " << inputname << endl;
      infile.precision(11);

      cout << "reading LX and  LY =";
      infile >> LX >> LY; // clasic 1 41
      cout << LX << " and " << LY << endl;
      
      cout << "reading NR_iterations and nr_samples =";
      infile >> NR_iterations >> nr_samples; // clasic 10000 10
      cout << NR_iterations << " and " <<  nr_samples << endl;
      
      cout << "kin_viscosity =";
      infile >> kin_visc ; // clasic 0.16
      cout << kin_visc << endl;
      
      cout << "Force.x and Force.y =";
      infile >> Force[0] >>  Force[1]; // clasic 1.0e-6 0.0
      cout << Force[0] << ", " <<  Force[1] << endl;
      cout << "Upper_Wall_velocity.x , Upper_Wall_velocity.y ";
      infile >> Upper_Wall_velocity[0] >>  Upper_Wall_velocity[1] ; // clasic 0.e-6 0.0
      cout << Upper_Wall_velocity[0] << Upper_Wall_velocity[1]  << endl;
      cout << "Lower_Wall_velocity.x and Lower_Wall_velocity.y";
      infile >> Lower_Wall_velocity[0] >>  Lower_Wall_velocity[1] ; // clasic 0.e-6 0.0
      cout << Lower_Wall_velocity[0] << Lower_Wall_velocity[1]  << endl;
      infile.close();
      cout << "read_parameter_file: end." << endl;
    }
}



int PeriodicBC(int a, int b)
{
   int xnew = a % b;
   if(xnew < 0)
   xnew+=b;
   return xnew;

}

void allocateMemory(void) {

  f = new double**[LX];
  f_temp = new double**[LX];

  f_eq_new = new double[beta];
  f_eq = new double[beta];

  rho = new double*[LX];
  u_x = new double*[LX];
  u_y = new double*[LX];
  du_x = new double*[LX];
  du_y = new double*[LY];
  obstacle =  new int*[LX];
  sigma = new double*[LX];

  for (int i=0;  i<LX; i++){

    u_x[i]= new double[LY];
    u_y[i]= new double[LY];
    rho[i]= new double[LY];
    du_x[i] = new double[LY];
    du_y[i] = new double[LY];

    sigma[i] = new double[LY];
    obstacle[i] = new int[LY];

    f[i] = new double*[LY];
    f_temp[i] = new double*[LY];

    for (int j=0;j<LY;j++){

      f[i][j]=new double[beta];
      f_temp[i][j]=new double[beta];
    }
  }
}


void CreateTwoPlanarWalls(void)
{

  //first initialize all nodes as non-obstacle
  cout << "CreateTwoPlanarWalls: Will create two planar walls..."<< endl;
  for(int i=0;i<LX;i++) {
    for(int j=0;j<LY;j++) {
      obstacle[i][j] = 0;
    }
  }

  //define obstacle nodes
  for(int i=0;i<LX;i++) {
    obstacle[i][0] = 1;  // 
    obstacle[i][LY-1] = 1;  
  }
  cout << "CreateTwoPlanarWalls: end."<< endl;
}

void computeEquilibriumDistribution(int i, int j)
{
  for (int k=0; k<beta ; k++)
    {
      f_eq[k] = WLIST[k]*rho[i][j]* (1.0+ 3.0*(u_x[i][j]*cX[k] + u_y[i][j]*cY[k]) + (9.0/2.0)*(u_x[i][j]*cX[k] + u_y[i][j]*cY[k])*(u_x[i][j]*cX[k] + u_y[i][j]*cY[k]) -(3.0/2.0)*(u_x[i][j]*u_x[i][j] + u_y[i][j]*u_y[i][j]) ) ;
    }
}

//This code makes use of the so called exact difference method. Within this approach, the effect of force in the collision step is accounted for via a shift of the equilibrium distribution around u to a new distribution around u+du, where du=F dt /rhom (= F/rho since dt=1). One thus adds feq(u+du)-feq(u) to the collision operator. We, therefore, evaluate here feq(u+du) and save it in f_eq_new. See also the routine Collision().
void computeNewEquilibriumDistribution(int i, int j)
{
  for (int k=0; k<beta ; k++)
    {
      f_eq_new[k] = WLIST[k]*rho[i][j]* (1.0+ 3.0*((u_x[i][j]+du_x[i][j])*cX[k] + (u_y[i][j]+du_y[i][j])*cY[k]) + (9.0/2.0)*((u_x[i][j]+du_x[i][j])*cX[k] + (u_y[i][j]+du_y[i][j])*cY[k])*((u_x[i][j]+du_x[i][j])*cX[k] + (u_y[i][j]+du_y[i][j])*cY[k]) -(3.0/2.0)*((u_x[i][j]+du_x[i][j])*(u_x[i][j]+du_x[i][j]) + (u_x[i][j]+du_x[i][j])*(u_y[i][j]+du_y[i][j]) )) ;
    }
}



void computeDensityandVelocity(void)
{
double sum = 0;
double sumX, sumY ;
for(int i=0;i<LX;i++)
	{
	for(int j=0; j<LY;j++)
		{
			sum = 0; sumX = 0; sumY = 0;
			for (int k=0; k<beta ; k++)
			{
			sum = sum + f[i][j][k];
			sumX = sumX + f[i][j][k]*cX[k];
			sumY = sumY + f[i][j][k]*cY[k];
			}

			rho[i][j] = sum;
			u_x[i][j] = (sumX )/sum;
			du_x[i][j]= Force[0]/sum;
			du_y[i][j]= Force[1]/sum;
			u_y[i][j] = (sumY)/sum;

		}
	}

}

/*void ComputeFLB (int i, int j)
{
  for (int k=0; k<beta ; k++)
    {
      
      flb[k]  = (1.0 - 0.5/tau)*WLIST[k]*rho[i][j]*(3.0*((cX[k] - u_x[i][j])*Force[0] + (cY[k] - u_y[i][j])*Force[1]) +  9.0*(u_x[i][j]*cX[k] + u_y[i][j]*cY[k])*(cX[k]*Force[0]+cY[k]*Force[1])) ;
      
    }
}
*/

void Collision(void)
{
  for(int i=0;i<LX;i++) {
    for(int j=0; j<LY;j++) {
      if( obstacle[i][j] != 1) {
      //apply collision for non-solid nodes only
    	computeEquilibriumDistribution(i,j);
    	computeNewEquilibriumDistribution(i,j);
    	//ComputeFLB (i,j);
    	for (int k=0; k<beta ; k++)
    //In the line below, (f_eq_new[k]-f_eq[k]) accounts for the effect of force within the exact difference method.
    	  f_temp[i][j][k] = f[i][j][k]*(1.0- 1.0/tau) + (1.0/tau)*f_eq[k] +  (f_eq_new[k]-f_eq[k]) ;
      }
    }
  }
}


void Propagation(void)
{
  for(int i=0;i<LX;i++)
    {
      for(int j=0; j<LY;j++)
	{
	  if( obstacle[i][j] != 1) //only non-obstacles send*/
	    {
	      for (int k=0; k<beta ; k++)
		{
		  int destinationX = PeriodicBC(i+cX[k], LX); //Periodic X Boundary
		  int destinationY = PeriodicBC(j+cY[k], LY); //Periodic Y Boundary
		  
		  //invert populations if destination is an obstacle
		  if( obstacle[destinationX][destinationY] == 1) 
		    {
		     	/*half way bounce back*/
		     	if (destinationY==0)
		     	{
		      	f[i][j][INVLIST[k]] = f_temp[i][j][k] - 2*WLIST[k]*rho[i][j]*(Upper_Wall_velocity[0]*cX[k]+Upper_Wall_velocity[1]*cY[k])*3;
		    	}
		    	else if (destinationY==(LY-1))
		    	{
		    		f[i][j][INVLIST[k]] = f_temp[i][j][k] - 2*WLIST[k]*rho[i][j]*(Lower_Wall_velocity[0]*cX[k]+Lower_Wall_velocity[1]*cY[k])*3;
		    	}
		    }
		    
		  else
		    {
		      f[destinationX][destinationY][k] = f_temp[i][j][k];
		    }
		}
	    }
	}
    }
}

void ComputeShearStress(void)
{
   for(int i=0;i<LX;i++)
	{
		for(int j=0; j<LY;j++)
		{
			double sum = 0;
			computeEquilibriumDistribution(i,j);
			for (int k=0; k<beta ; k++)
			{
				sum = sum + (f[i][j][k] - f_eq[k])*cX[k]*cY[k];	
			}
		   sigma[i][j] = -(1 - .5/tau) * sum;	
		} 
	}
}

void writeData(int time)
{

sprintf(dens, "obs/density_field_t%d.dat",time);
sprintf(shearstress, "obs/shearstress_field_t%d.dat",time);
sprintf(velX, "obs/velocity_field_t%d.dat",time);

densityFile = fopen(dens, "w");
shearFile = fopen(shearstress, "w");
veloFile = fopen(velX, "w");
 for (int i = 0; i < LX; i++)
   {
     for (int j = 0; j < LY; j++)
       {
	 if(obstacle[i][j]!=1) {
	   fprintf(densityFile, "%d %d %e\n", i,  j, rho[i][j]  );
	   fprintf(shearFile, "%d %d %e\n", i,  j, sigma[i][j]  );
	   fprintf(veloFile, "%d %d %e %e\n", i,j, u_x[i][j], u_y[i][j]);
	 }
       }
   }
 fclose(densityFile);
 fclose(shearFile);
 fclose(veloFile);

 int i=LX/2;
 sprintf(dens, "obs/density_profile_x%d_t%d.dat",  i,  time);
 sprintf(shearstress, "obs/shearstress_profile_x%d_t%d.dat",i,time);
 sprintf(velX, "obs/velocity_profile_x%d_t%d.dat",  i, time);
densityFile = fopen(dens, "w");
shearFile = fopen(shearstress, "w");
veloFile = fopen(velX, "w");
// for (int i = 0; i < LX; i++)
   {
     for (int j = 0; j < LY; j++)
       {
	 if(obstacle[i][j]!=1) {
	   fprintf(densityFile, "%d %e\n", j, rho[i][j]  );
	   fprintf(shearFile, "%d %e\n", j, sigma[i][j]);
	   fprintf(veloFile, "%d %e %e\n", j, u_x[i][j], u_y[i][j]);
	 }
       }
   }
 fclose(densityFile);
 fclose(shearFile);
 fclose(veloFile);
}


// MAIN FUNCTION

int main()
{

double sum = 0;

 char obstacle_filename[200];
 char parameter_filename[200];


 sprintf(obstacle_filename, "obstacles.dat");
 sprintf(parameter_filename, "parameters.dat");

// read LX, LY, LZ, simulation_time, viscosity, gravity,  etc....
 read_parameter_file(parameter_filename);

 if(nr_samples>0) {
   dt_save = (int)(NR_iterations/nr_samples);
 }else{
   cout << "nr_samples must be > 0!: nr_samples=" << nr_samples << endl;
   cout << " ==> Exitting to the system..." << endl;
   exit(13);
 }

 tau = 3.0*kin_visc + 0.5 ;
 allocateMemory();


  CreateTwoPlanarWalls();
  
//initializating density and velocity
cout<< "intializing desity and velocity "<<endl;
for(int i=0;i<LX;i++)
  {
    for(int j=0; j<LY;j++)
      {
      	rho[i][j] = 1.0; // density 
      	u_x[i][j] = 0.0; // velocity in x direction
      	u_y[i][j] = 0.0; // velocity in y direction
      }
  }
cout<< "intializing desity and velocity done. "<<endl;


 //set f to f_eq for initialization
cout<< "setting f to f_eq for intialization "<<endl;
  for(int i=0;i<LX;i++){
    for(int j=0; j<LY;j++){

      computeEquilibriumDistribution(i,j);

      for (int k=0; k<beta ; k++){             
          f[i][j][k]    =   f_eq[k] ;
          f_temp[i][j][k] = f_eq[k] ;
      }
    }
  }
cout<< "setting f to f_eq for intialization done. "<<endl;


 //remove already existing files in the folder in case we are running the code again
(void) system("if [ -d obs ] ; then rm -rf obs; fi");
(void) system("mkdir obs");

for(int iter=0 ; iter <NR_iterations+1; iter ++ )
  {
  	
  	 if(iter%dt_save ==0)
      {
    		cout << " main: computing Density and velocity at t=" << iter<<endl;
    	}
    computeDensityandVelocity();
    if(iter%dt_save ==0)
      {
    		cout << " main: computing collission at t=" << iter<<endl;
    	}
    Collision();
    
    if(iter%dt_save ==0)
      {
    		cout << " main: computing propation at t=" << iter<<endl;
      }
    Propagation();
    ComputeShearStress();
    
    if(iter%dt_save ==0)
      {
    		cout << " main: saving results on disk for t=" << iter<<endl;    
			writeData(iter);
      }
  }

 return 0;
}




