import numpy as np
import open3d as o3d
import pandas as pd

def load_lidar_timestamps():
    np.printoptions(suppress=True)

    # file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/lidar/lidar_timestamps.txt'
    file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/map_10km_post_processed/lidar/lidar_timestamps.txt'

    lidar_timestamps_string = np.loadtxt(file, dtype=str)
    lidar_timestamps = lidar_timestamps_string.astype(np.float)
    lidar_timestamps = lidar_timestamps.reshape(-1, 2)
    
    # print ("lidar_timestamps is: ", lidar_timestamps)
    # print ("lidar_timestamps[6895] is: ", lidar_timestamps[6895])
    # print ("lidar_timestamps.shape is: ", lidar_timestamps.shape)
    # print ("\n")
    return lidar_timestamps

lidar_timestamps = load_lidar_timestamps()

# Convert the timestamps into a DataFrame
df = pd.DataFrame(lidar_timestamps, columns=['timestamp', 'id'])
# Save the DataFrame to CSV
# df.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/lidar_timestamps.csv', index=False)
df.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/lidar_timestamps_new.csv', index=False)
