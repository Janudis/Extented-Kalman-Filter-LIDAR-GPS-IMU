import pandas as pd

# Read the two CSV files
# , low_memory=False
# imu_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/imu_read_data/merge_lidar_imu.csv')
# imu_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/imu_read_data/LIDAR_IMU2.csv')

# imu_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/imu_read_data/LIDAR_IMU.csv')
# distance_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/output_gps_data.csv')
imu_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/imu_read_data/LIDAR_IMU_new.csv')
# imu_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/imu_read_data/LIDAR_IMU_new2.csv')
distance_data = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/output_gps_data_new.csv')
# Merge the DataFrames on the 'timestamp' column using an outer join
merged_data = pd.merge(imu_data, distance_data, on='timestamp', how='outer')

# Sort by 'timestamp'
merged_data = merged_data.sort_values('timestamp')

# Save the merged data to a new CSV file
# merged_data.to_csv('final_merge.csv', index=False)
# merged_data.to_csv('final_merge2.csv', index=False)
merged_data.to_csv('final_merge_new.csv', index=False)
# merged_data.to_csv('final_merge_new2.csv', index=False)

# Specify the desired column order
column_order = [
    'timestamp', 'easting', 'northing', 'altitude', 'speed',
    'imu_ax', 'imu_ay', 'imu_az', 'imu_roll', 'imu_pitch', 'imu_yaw',
    'imu_roll_rate', 'imu_pitch_rate', 'imu_yaw_rate',
    'x', 'y', 'z'
]
# Reorder the columns according to the specified order
merged_data = merged_data[column_order]

# Delete the first 12496 lines
merged_data = merged_data.iloc[13071:73573]

# merged_data = merged_data.drop(merged_data.index[0:1587])

# Save the merged data to a new CSV file
# merged_data.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/reordered_final_merge.csv', index=False) #VALTO KALITERA STO CMAKE ME TIN MIA
# merged_data.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/reordered_final_merge2.csv', index=False) #VALTO KALITERA STO CMAKE ME TIN MIA
merged_data.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/reordered_final_merge_new.csv', index=False) #VALTO KALITERA STO CMAKE ME TIN MIA
# merged_data.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/reordered_final_merge_new2.csv', index=False) #VALTO KALITERA STO CMAKE ME TIN MIA

# # 2 KALMAN GIA TO STATE_YAW
# # Read the CSV files
# # df_reorder = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/reordered_final_merge2.csv')
# # df_output = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/gps_imu_output.csv')
# df_reorder = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/reordered_final_merge.csv')
# df_output = pd.read_csv('D:/Python_Projects/Extented-Kalman-Filter/final_ekf_without_Eigen/cmake-build-debug/output_GPS_IMU.csv')

# # Merge data based on the timestamp, keeping all the rows of df_reorder and only 'state_yaw' from df_output
# merged_df = pd.merge(df_reorder, df_output[['timestamp', 'state_yaw']], on='timestamp', how='left')

# # merged_df = merged_df.drop(merged_df.index[0:1587])

# # Write the updated data back to reordered_final_merge.csv
# merged_df.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/reordered_final_merge_2_ekfs.csv', index=False)
# # merged_df.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/reordered_final_merge_2_ekfs2.csv', index=False)
# # print(type(merged_df['x'].iloc[1810]))
