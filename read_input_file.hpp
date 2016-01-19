/***  reading of input file with obstacles  ***/

#include <fstream>
#include <iostream>

using namespace std;

void read_obstacle_file(char inputname[]){
	int x,y;


	ifstream infile;

	for (x=0;x<LX)
		for (y=0;y<LY)
			obstacle[x][y];

	infile.open(inputname);
	if(!infile){
		cout << "read_obstacle_file: Could not open " << inputname << endl;
		cout << " ==> Exiting system " << endl;
		exit(13); // ?
	}

	if (infile.is_open()){
		cout << "read_obstacle_file: reading position of obstacles nodes from " << inputname << endl;

		while (!infile.eof()){
			infile >> 
		}
	}
}
