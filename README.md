## Udacity - Self Driving Car Nanodegree (Term 2) - Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Background
---
In this project, my goal is to build an extended kalman filter in C++ that will work with the radar and lidar dataset provided to estimate the correct position of a vehicle in a simulation.

Overview of Repository
---
This repository contains the following source files that I have forked from the [main repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) and subsequently modified for the extended Kalman filter project:

1.  [FusionEKF.cpp](https://github.com/MartinKan/CarND-Extended-KF/blob/master/src/FusionEKF.cpp)
2.  [FusionEKF.h](https://github.com/MartinKan/CarND-Extended-KF/blob/master/src/FusionEKF.h)
3.  [kalman_filter.cpp](https://github.com/MartinKan/CarND-Extended-KF/blob/master/src/kalman_filter.cpp)
4.  [kalman_filter.h](https://github.com/MartinKan/CarND-Extended-KF/blob/master/src/kalman_filter.h)
5.  [tools.cpp](https://github.com/MartinKan/CarND-Extended-KF/blob/master/src/tools.cpp)
6.  [tools.h](https://github.com/MartinKan/CarND-Extended-KF/blob/master/src/tools.h)

The FusionEKF class initializes the Kalman filter and the variables that are used by the filter.  It calls functions in the Kalman filter that implement the prediction step or update step.  Depending on the type of sensor information it received, it will either call the standard Kalman filter update step (lidar) or the extended Kalman filter update step (radar).  When FusionEKF is called for the first time by main, it will try to initialize the object's location (contained inside the state vector x) with the sensor measurement.

The kalman_filter class contains functions for the prediction step as well as the Kalman filter update step (lidar) and extended Kalman filter update step (radar). Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. 

The tool class is a helper class that implements functions to calculate the root mean squared error and the Jacobian matrix.

To run the code, first compile the code in the build directory by running the following command:

	cmake .. && make

Then run the code by executing the following command:

	./ExtendedKF
