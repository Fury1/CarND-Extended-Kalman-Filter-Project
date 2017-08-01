# CarND-Extended-Kalman-Filter-Project

This is the first project of the Udacity Self Driving Car ND in term 2. In this repo is an Extended Kalman Filter implementation that fuses radar and lidar data to predict the most accurate position of a vehicle along a noisy measurement path.

## Dependencies

cmake >= 3.5
All OSes: click here for installation instructions
make >= 4.1
Linux: make is installed by default on most Linux distros
Mac: install Xcode command line tools to get make
Windows: Click here for installation instructions
gcc/g++ >= 5.4
Linux: gcc / g++ is installed by default on most Linux distros
Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
Windows: recommend using MinGW
Basic Build Instructions

Clone this repo.
Make a build directory: mkdir build && cd build
Compile: cmake .. && make
On windows, you may need to run: cmake .. -G "Unix Makefiles" && make
Run it: ./ExtendedKF path/to/input.txt path/to/output.txt. You can find some sample inputs in 'data/'.
eg. ./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt
