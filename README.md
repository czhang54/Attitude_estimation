# Attitude_estimation

This repository contains C++ codes for simulating nonlinear filters for attitude estimation. 
It is designed to be flexible for the user to add:
  + Target models
  + Sensor models
  + Filter algorithms
  
I am continuously working to make this project serve as a library for more general purposes. 

For an overview of relevant attitude filters, please refer to the tutorial article attitude_filter.pdf

Directories:
  + include: contains all the headers
  + src: contains all the source files
  + src/Filters: contains implementation of all attitude filters
  + test: contains test programs to do some demo simulations
 
Requirement:
  + Eigen: a convenient matrix library that can be downloaded at http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
 
The project was tested on Mac OS 10.12 using g++ 4.2.1.
To compile and run the project in terminal: 
```
  cd test
  g++ -std=c++11 -I ./../include -c ./../src/*.cpp test.cpp
  g++ *.o -o test
  ./test
```

Alternatively, one can build/compile the project using the provided CMakeLists.txt:
```
  mkdir build
  cd mkdir 
  cmake -G "Unix Makefiles" ..    
  make    
  ./attitude_estimation    
```
Users may make substitutions (project name CMake file type) in the above command lines as appropriate.

