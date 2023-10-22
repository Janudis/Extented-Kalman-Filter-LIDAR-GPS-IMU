import pandas as pd
import numpy as np
import math

""""
merge imu-gps for testing the ekf only gps-imu
"""

# Read the two CSV files
# imu_data = pd.read_csv('all_imu_data.csv')
# gps_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/gps_read_data/gps_data.csv')
gps_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/gps_read_data/gps_data_new.csv')
imu_data = pd.read_csv('all_imu_data_new0.csv')

# Merge the DataFrames on the 'timestamp' column using an outer join
merged_data = pd.merge(gps_data, imu_data, on='timestamp', how='outer')

# Sort by 'timestamp'
merged_data = merged_data.sort_values('timestamp')

# Save the merged data to a new CSV file
merged_data.to_csv('GPS_IMU_new.csv', index=False)
merged_data.to_csv('D:/Python_Projects/Extented-Kalman-Filter/final_ekf_hav_cpp/cmake-build-debug/GPS_IMU_new.csv', index=False)
