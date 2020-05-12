[//]: # (Image References)

[image1]: ./report/Dataset1.jpg "Dataset 1 Simulation"
[image2]: ./report/Dataset2.jpg "Dataset 2 Simulation"


## Self Driving Car Nano Degree 
## Extended Kalman Filter Project

### The Project

This project aims to build an extended kalman filter to estimate the state of a moving vehicle using a combination of radar and laser measurements. The laser sensor provides a cartesian position of the vehicle with a noisy estimate of velocity possible from the difference between sequential measurements. The radar sensor provides polar-style radial distance and angle but also benefits from a radial velocity measurement. Both sensor measurements can arrive asynchronously and with varying degrees of error. 

The completed filter will use a standard Kalman filter for laser measurements and an extended kalman filter for the radar measurements. It will be tested using the Term 2 Simulator (available [here](https://github.com/udacity/self-driving-car-sim/releases)). 

### Build and Install

As the project is presented in the Udacity Workspace, all dependencies are already installed and the filter can be built and run using the following commands:

`mkdir build
cd build
cmake ..
make
./ExtendedKF`

This will build and execute the filter which will then wait for a connection (using uWebSocketIO) to the simulator. 

The simulator will provide sensor measurements and the filter program will output the following:

* X position estimate
* Y position estimate
* Root Mean Square X error
* Root Mean Square Y error
* Root Mean Square X velocity error
* Root Mean Square Y velocity error

### Results 

The images below show the results from the two test data sets available in the simulator. The green output shows the filter estimate which provides a very good approximation of the ground truth.

![alt text][image1]

![alt text][image2]

