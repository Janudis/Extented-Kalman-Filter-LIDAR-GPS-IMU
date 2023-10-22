import pandas as pd
import numpy as np
import math

# Read the two CSV files
# imu_data = pd.read_csv('all_imu_data.csv')
imu_data = pd.read_csv('all_imu_data_new0.csv')
# distance_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/distance_data.csv')
distance_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/distance_data_new.csv')
# distance_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/distance_data_new2.csv')

# Merge the DataFrames on the 'timestamp' column using an outer join
merged_data = pd.merge(imu_data, distance_data, on='timestamp', how='outer')

# Sort by 'timestamp'
merged_data = merged_data.sort_values('timestamp')

# Save the merged data to a new CSV file
merged_data.to_csv('LIDAR_IMU_new.csv', index=False)
# merged_data.to_csv('LIDAR_IMU_new2.csv', index=False)
# merged_data.to_csv('LIDAR_IMU2.csv', index=False)

# def degrees_to_radians(degrees):
#     return degrees * (math.pi / 180.0)

# def radians_to_degrees(radians):
#     return radians * (180.0 / math.pi)

# def modify_bearing(input_csv_path, output_csv_path):
#     # Read the CSV file into a DataFrame
#     df = pd.read_csv(input_csv_path)

#     # Initialize variables to store the last non-null imu_yaw value
#     last_imu_yaw = None
#     last_index = None

#     for index, row in df.iterrows():
#         # Check if imu_yaw is not null
#         if pd.notnull(row['imu_yaw']):
#             last_imu_yaw = row['imu_yaw']
#             last_index = index
#         else:
#             # If imu_yaw is null, use the last non-null imu_yaw value
#             # df.at[index, 'bearing'] = df.at[index, 'bearing'] + degrees_to_radians(last_imu_yaw)
#             df.at[index, 'bearing'] = -df.at[index, 'bearing'] + degrees_to_radians(last_imu_yaw)


#     # Save the updated DataFrame to a new CSV file
#     df.to_csv(output_csv_path, index=False)

# # Example usage
# input_csv_path = 'LIDAR_IMU.csv'
# output_csv_path = 'merge_lidar_imu.csv'
# modify_bearing(input_csv_path, output_csv_path)







