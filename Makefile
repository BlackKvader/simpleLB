BOOST = -IC:/boost_1_60_0/
# EIGEN = -IC:/_Peto/programing/eigen/
EIGEN = -I/home/peter.sichman/eigen3.3/


all:
	echo ""
	g++ -Wall main.cpp Tsystem.hpp Tsystem.cpp Tpoint.hpp Tpoint.cpp global.cpp global.hpp -o main.exe
 
# this work just on not mounted computers

main: main.o Tsystem.o global.o
	g++ -Wall main.o Tsystem.o -o main.exe

#use -v for more info in compiling

main.o: main.cpp Tsystem.o global.o
	g++ -Wall -c main.cpp Tsystem.o global.o

Tsystem.o: Tsystem.hpp Tsystem.cpp Tpoint.hpp global.o
	g++ -Wall -c Tsystem.hpp Tsystem.cpp Tpoint.hpp Tpoint.cpp global.o

global.o: global.hpp global.cpp
	g++ -Wall -c global.cpp global.hpp

clean:
	rm *.o || echo "No *.o files. Continuing clean"
	rm main.exe || echo "No main.exe file. Continuing clean"
