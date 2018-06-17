# Extended Kalman Filter Project
[//]: # (Image References)
[result]:   ./result.png

Self-Driving Car Engineer Nanodegree Program

In this project I implemented a Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

This project utilizes the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Data from both sensors (radar and lidar) is fused to improve the estimation accuracy of a moving object subject to noisy measurements and uncertain trajectory.
The moving object is modeled with a linear system where the state vector is composed by position and velocity in the x and y axes, it is assumed that the object moves in straight line with constant velocity. This assumption holds as long as the changes in direction and velocity are not excessive and sudden (that is, as long as the linearized trajectory around the current position is a reasonable approximation of the true trajectory). Another important assumption is that the noises are gaussian.
The steps necessary to obtain the object state estimation are (as implemented in the `FusionEKF.ProcessMeasurement()` method):
1) Predict position and velocity of the object using the current belief on our system. This will be subject to uncertainty given by the process covariance matrix.
2) Measure the position and velocity using sensors, each sensor is not ideal and will have noise in the measurement. The uncertainty is given by the measurement covariance matrix.
3) The estimated position and velocity will combine the states from 1) and 2) using well known equations.
The result is a position and velocity estimate with significantly less uncertainty of both model and measurements.

Linear Kalman filter equations are used for lidar measurements, and the non-linear version of the Kalman filter (Extended Kalman filter) is used to estimate the state vector from radar measurements (since the state transition matrix is non linear, due to polar coordinates). The main difference between the "standard" linear Kalman Filter and the Extended Kalman filter is that the state transition matrix H in the EKF, used to calculate the other matrixes in the update step, is a linearization around the current state of the state transition function h(x). Such linearization is performed with a first order Taylor expansion implemented in the `Tools.CalculateJacobian` method. However, the prediction still uses the non-linear state transition function h(x) based on the current knowledge of the state of the object.
 
The flow is implemented in the file `FusionEKF.cpp`, whereas the Kalman filter equations are in `kalman_filter.cpp`. To measure the accuracy of the algorithm, a Root Mean Square Error function is implemented in `tools.cpp`.
Using the simulator it is possible to visualize the result of sensor fusion algorithm.

![alt text][result]

Blue and red dot are measurements from lidar and radar, while the green dots are the estimated position of the object (represented by the car), as detected by the sensors placed at the origin.
The image also shows the RMSE of position and velocity, which were required to be lower than [0.11, 0.11, 0.52, 0.52].


The code is based on the [Udacity repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).
The repository contains further details on how to setup the development and build environment.
