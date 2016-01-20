/*** fast programing version of lattice boltzman ***/

#include <iostream>

// #include "read_input_file.hpp"
#include "Tsystem.hpp"

using namespace std;

int main(){
	Tsystem my_system;

	my_system.init();
	my_system.run();
	my_system.check();
	// my_system.close();

	cout << "I'm done!" <<endl;
	return 0;
}
