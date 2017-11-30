# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project, a kalman filter has been build to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

## Structure of the source code

All the codes are stored at src/ folder, which contains:
- Main.cpp: 
  - Handles the uWebsocketIO communication between the simulator and it's self.
  - Reads in the sensor data and calls FusionEKF to process the data
  - Stores the data into a measurement object that it passes to the Kalman filter for processing
  - Also a ground truth list and an estimation list are used for tracking RMSE.
- FusionEKF.h & .cpp:
  - Initialize Variables as taught in the class:
  ```
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  ```
  - The ProcessMeasurement() function initilize EKF instance using the first measurement received, and then predict & update by using the functions in kalman_filter.cpp
  
- kalman_filter.h & .cpp：
  - Implement the Predict() & Update() & UpdateEKF() functions by using what we have learnt in the class with Andrei
  - Notable, in UpdateEKF(), I used a helper function NormlizeAng() to normalize a theta into range of [-pi, pi]

- tools.cpp:
  - Calculates the RSME
  - Calculates the Jacobian Matrix based on current state
