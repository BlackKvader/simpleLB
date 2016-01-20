/*** main system class ***/

#ifndef TSYSTEM_HPP
#define TSYSTEM_HPP

#include <cstring>
#include <iostream>

#include "Tpoint.hpp"

using namespace std;

class Tsystem{
public:
	// Tpoint **point;
	// OK. I give up. But it was working for while ...
	Tpoint point[41][1];

	// functions
	void read_parameter_file(const string parameter_filename); // probably need const *char ...
	void run();
	void init();
	void check();
	void close();
	void write(int num);
};

#endif
