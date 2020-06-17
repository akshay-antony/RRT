# rrt
Basic implementation of rrt algorithm using circular obstacles

Steps for running the file

$ git clone https://github.com/akshay-antony/rrt.git

$ cd rrt

$ g++ -c rrt.cpp 
 
$ g++ -c main.cpp 

$ g++ -o rrt  rrt.o main.o

$ ./rrt

For Plotting

$ gnuplot

$ plot "result.dat" using 1:2 with linespoint   //Note results.dat is generated in /home/akshay/results.dat for my computer you can edit that on line 122 of main.cpp

Major Points in the program

Destination is assumed to be circle with given input and radius, radius usually for this program r>0.1

Obstacles are assumed to be circular

