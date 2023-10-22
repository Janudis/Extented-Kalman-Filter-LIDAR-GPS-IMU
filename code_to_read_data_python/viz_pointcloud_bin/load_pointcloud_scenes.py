import numpy as np
import open3d as o3d
import pandas as pd
import math

def vis_pointcloud_mine(pcd_id):
     ### For KITTI ###

#    file = 'data/000035.bin'
    # file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/lidar/'+str(pcd_id)+'.bin'

    pcd_float = np.fromfile(file, dtype=np.float32).reshape(-1, 4)
    # print ("pcd_float is: ", pcd_float)
    # print ("pcd_float.shape is: ", pcd_float.shape)
    # print ("\n")

    intensity = pcd_float[:,3]
    pointcloud = pcd_float[:,0:3]
    # Filter points with intensity greater than 0.8
    filtered_points = pointcloud[intensity > 0.8]
    print("filtered_points:", filtered_points)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(filtered_points)
    # print ('pcd is:', pcd)
    o3d.visualization.draw_geometries([pcd])

def load_pointcloud_mine(pcd_id):
    """"
    thresholding: intensity > 0.8
                  (distance > 4) & (distance < 35)
                  z>0
                  Filter rows where point_count is more than 7  
    """

#    file = 'data/000035.bin'
    # file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/lidar/'+str(pcd_id)+'.bin'
    file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/map_10km_post_processed/lidar/'+str(pcd_id)+'.bin'

    pcd_float = np.fromfile(file, dtype=np.float32).reshape(-1, 4)
    # print ("pcd_float is: ", pcd_float)
    # print ("pcd_float.shape is: ", pcd_float.shape)
    # print ("\n")

    intensity = pcd_float[:,3]
    pointcloud = pcd_float[:,0:3]
    #print(intensity.max())
    # print ("intensity is: ", intensity)
    # print ("pointcloud is: ", pointcloud)
    # print ("pointcloud.shape is: ", pointcloud.shape)
    # print ("\n")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)
    # print ('pcd is:', pcd)
    #o3d.visualization.draw_geometries([pcd])

    # Filter points with intensity greater than 0.8
    filtered_points = pointcloud[intensity > 0.8]
    filtered_intensity = intensity[intensity > 0.8]
    
    # Compute the bearing of the filtered points
    # bearing = np.arctan2(filtered_points[:, 0], filtered_points[:, 1])
    # Calculate the distance of each point from the lidar sensor
    # distance = (filtered_points[:, 0]**2 + filtered_points[:, 1]**2 + filtered_points[:, 2]**2)**0.5
    distance = (filtered_points[:, 0]**2 + filtered_points[:, 1]**2)**0.5

    # Filter points based on distance and z-coordinate
    mask = (distance > 4) & (distance < 35) & (filtered_points[:, 2] > 0)
    filtered_points = filtered_points[mask]
    filtered_intensity = filtered_intensity[mask]

    # Get the number of points in the current point cloud
    num_points = len(filtered_points)

    # Convert the filtered points into DataFrame
    df = pd.DataFrame(filtered_points, columns=['x', 'y', 'z'])
    df['intensity'] = filtered_intensity
    df['distance'] = distance[mask]
    # df['bearing'] = bearing[mask]
    df['pcd_id'] = pcd_id
    df['point_count'] = num_points

    # Filter rows where point_count is more than 10
    filtered_df = df[df['point_count'] > 7]
    # Append the DataFrame to the CSV file
    # filtered_df.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/filtered_points.csv', mode='a', index=False, header=False)
    filtered_df.to_csv('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/filtered_points_new.csv', mode='a', index=False, header=False)

# pcd_id = 5
# vis_pointcloud_mine(pcd_id)

# Write the header for the CSV file
# with open('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/filtered_points.csv', 'w') as file:
with open('D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/filtered_points_new.csv', 'w') as file:
    file.write('x,y,z,intensity,distance,pcd_id,point_count\n')

# You can change the range as per your requirements
# for pcd_id in range(0, 6906):
for pcd_id in range(0, 19318):
    load_pointcloud_mine(pcd_id)

# pcd_id with the biggest point_count: 1577
# Read the existing CSV file
# csv_file_path = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/filtered_points.csv'
csv_file_path = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/filtered_points_new.csv'
df = pd.read_csv(csv_file_path)

if not df.empty:
    # Find the pcd_id with the biggest point_count
    max_point_count_row = df[df['point_count'] == df['point_count'].max()]
    max_pcd_id = max_point_count_row['pcd_id'].values[0]

    print("pcd_id with the biggest point_count:", max_pcd_id)
else:
    print("The CSV file is empty. There are no points in the DataFrame.")