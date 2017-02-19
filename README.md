# Attitude_estimation

This repository contains C++ codes for simulating nonlinear filters for attitude estimation. 
It is designed to be flexible for the user to add:
  + Target models
  + Sensor models
  + Filter algorithms
  
I am continuously working to make this project serve as a library for more general purposes.

Directories:
  + include: contains all the headers
  + src: contains all the source files
  + example: contains test programs to do simulations
 
Requirement:
  + Eigen: a convenient matrix library that can be downloaded at http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
 
The project was tested on Mac OS 10.12 using g++ 4.2.1.
To compile and run the project in terminal: 
  + cd example
  + g++ -std=c++11 -I ./../include -c ./../src/*.cpp example.cpp
  + g++ *.o -o example
  + ./example

For an overview of relevant attitude filters, please refer to the tutorial article attitude_filter.pdf
