# Sensor Fusion of LIDAR, GPS and IMU with Extended Kalman Filter for Localization in Autonomous Driving
![alt text](https://github.com/Janudis/EKF_GPS_IMU/blob/master/Extended-Kalman-Filter-Step.png)
   
This code implements an Extended Kalman Filter (EKF) for fusing Global Positioning System (GPS), Inertial Measurement Unit (IMU) and LiDAR measurements. The goal is to estimate the state (position and orientation) of a vehicle.

# Algorithm
1) Initialization: Firstly, initialize your EKF with initial estimates for the position, velocity, and orientation, typically using your first GPS reading for position and velocity. The covariance matrix (P) should also be initialized to reflect initial uncertainty.
2) Prediction step (also known as Time Update): In the prediction step, you make a prediction of the current state using the process model and the previously estimated state. In our case, the state can be represented as [position, velocity, orientation]. We also need to predict the state covariance matrix (P) at this point.
3) Update step (also known as Measurement Update): In the update step, we correct the predicted state using the measurement data. GPS frequency is 4 Hz and LiDAR frequency is 10 Hz.
4) Estimate update: With the Kalman gain, we can compute the updated (a posteriori) state estimate by combining the predicted state estimate and the weighted difference between the actual measurement and the measurement predicted by the a priori estimate (also known as the measurement residual).  
5) Covariance update: We also compute the updated (a posteriori) estimate covariance (P). The a posteriori state and covariance estimates at the current time become the a priori estimates for the next time step.
6) Repeat steps 2-5: This process is then repeated for each time step, using the a posteriori estimates from the previous time step as the a priori estimates for the current step.  

# Dependencies
C++ compiler supporting C++11 or higher

# Usage
1) Install the required dependencies and ensure they are properly linked in your build environment.
2) Place the input data file in the same directory as the code files.
3) Compile the code using a C++ compiler.
4) Change the CMakeLists.txt before running the compiled executable.
5) The estimated position and orientation will be saved in the output csv file.

# Code Structure
[Similar to ](https://github.com/Janudis/Extended-Kalman-Filter-GPS_IMU/tree/master) with some additions like:

1) lidar_preprocessing : read and apply thresholds in lidar pointcloud
2) DBSCAN_clustering : do DBSCAN clustering and return the closest's cluster centroid
3) gps_geodetic_utm : GPS data from geodetic to UTM coordinate system
4) lidar_gt_geodetic_utm : lidar groundtruth landmarks from geodetic to UTM coordinate system
5) data_preprocessing : association of observed landmarks with the ground-truth landmarks (used in main)

# Input Data Format
The input data is expected to be in a CSV file with the following columns:

Timestamp (in nanoseconds)

Latitude (in UTM 35)

Longitude (in UTM 35)

Altitude (in UTM 35)

Forward velocity (in meters per second)

Yaw rate (in radians per second)

# Map Data Format
Since we work with LiDAR we need a map of the landmarks. The landmarks data are stored in lidar_gt_rtk_utm2.csv with the following columns:

Latitude (in UTM 35)

Longitude (in UTM 35)

# Adjusting Parameters
The code provides options for adjusting the standard deviation of the observation noise for x and y coordinates (xy_obs_noise_std), the yaw rate (yaw_rate_noise_std), and the forward velocity (forward_velocity_noise_std). These parameters can be modified in the code to suit your specific scenario and sensor characteristics.
