import numpy as np
import pandas as pd
import csv

def dbscan_clustering(points, eps, min_samples):
    """
    Apply DBSCAN clustering to a set of points.
    
    :param points: A numpy array of shape (n, 3) representing the filtered points.
    :param eps: The maximum distance between two samples for them to be considered as in the same neighborhood.
    :param min_samples: The number of samples in a neighborhood for a point to be considered as a core point.
    :return: An array of cluster labels, where -1 represents noise points.
    """
    n = points.shape[0]
    cluster_labels = np.zeros(n, dtype=int)  # Initialize cluster labels with zeros
    
    cluster_id = 1  # Start with cluster id 1
    
    for i in range(n):
        if cluster_labels[i] != 0:
            continue  # Point already assigned to a cluster or noise
        
        # Find neighboring points within eps distance
        distances = np.linalg.norm(points - points[i], axis=1)
        neighbors = np.where(distances <= eps)[0]
        
        if len(neighbors) < min_samples:
            cluster_labels[i] = -1  # Label point as noise
        else:
            cluster_labels[i] = cluster_id  # Assign a new cluster id
            
            # Expand the cluster
            while len(neighbors) > 0:
                current_point = neighbors[0]
                
                if cluster_labels[current_point] == 0:
                    cluster_labels[current_point] = cluster_id
                    distances = np.linalg.norm(points - points[current_point], axis=1)
                    new_neighbors = np.where(distances <= eps)[0]
                    
                    if len(new_neighbors) >= min_samples:
                        neighbors = np.concatenate((neighbors, new_neighbors))
                
                neighbors = neighbors[1:]
            
            cluster_id += 1  # Move to the next cluster id
    
    return cluster_labels

def points_with_clusters(points, cluster_labels):
    """
    Create a list of points paired with their cluster labels.
    
    :param points: A numpy array of shape (n, 3) representing the points.
    :param cluster_labels: An array of cluster labels for each point.
    :return: A list of tuples where each tuple contains a point and its cluster label.
    """
    points_with_clusters_list = [(point, cluster_label) for point, cluster_label in zip(points, cluster_labels)]
    return points_with_clusters_list

def find_closest_cluster_centroid(clustered_points):
    """
    Find the cluster centroid closest to a reference point.
    
    :param clustered_points: A list of tuples containing points and cluster labels.
    :param reference_point: The reference point to measure distances from.
    :return: The centroid of the closest cluster.
    """
    cluster_centroids = {}
    centroids_list = []
    reference_point = np.array([0, 0, 0])  

    for point, cluster_label in clustered_points:
        if cluster_label != -1:
            if cluster_label in cluster_centroids:
                cluster_centroids[cluster_label].append(point)
            else:
                cluster_centroids[cluster_label] = [point]
    
    min_distance = float('inf')
    closest_cluster = None
    
    for cluster_label, points_in_cluster in cluster_centroids.items():
        centroid = np.mean(points_in_cluster, axis=0)
        centroids_list.append(centroid)
        distance = np.linalg.norm(centroid - reference_point)

        if distance < min_distance:
            min_distance = distance
            closest_cluster = centroid
    
    return closest_cluster
    # return centroids_list


def process_csv(input_file, output_file, eps, min_samples):
    # Read the input CSV file and store its contents
    with open(input_file, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)  # Skip the header row
        data = [list(map(float, row)) for row in csv_reader]

    # Group points by unique pcd_id
    data_dict = {}
    for row in data:
        pcd_id = int(row[5])
        if pcd_id in data_dict:
            data_dict[pcd_id].append(row)
        else:
            data_dict[pcd_id] = [row]

    # Process each unique pcd_id
    # results = {}
    results = []
    for pcd_id, points_data in data_dict.items():
        points = np.array(points_data)[:, :3]  # Extract x, y, z columns
        cluster_labels = dbscan_clustering(points, eps, min_samples)
        points_clustered = points_with_clusters(points, cluster_labels)
        closest_cluster_centroid = find_closest_cluster_centroid(points_clustered)
        # cluster_centroids = find_closest_cluster_centroid(points_clustered)

        if closest_cluster_centroid is not None:
            centroid_x, centroid_y, centroid_z = closest_cluster_centroid
            results.append([pcd_id, centroid_x, centroid_y, centroid_z])

        # for centroid in cluster_centroids: #axristo
        #     centroid_x, centroid_y, centroid_z = centroid
        #     results.append([pcd_id, centroid_x, centroid_y, centroid_z])

        # x_values = []
        # y_values = []
        # z_values = []

        # for centroid in cluster_centroids:
        #     centroid_x, centroid_y, centroid_z = centroid
        #     x_values.append(str(centroid_x))
        #     y_values.append(str(centroid_y))
        #     z_values.append(str(centroid_z))

        # results[pcd_id] = (','.join(x_values), ','.join(y_values), ','.join(z_values))

    # Write the results to the output CSV file
    with open(output_file, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['pcd_id', 'x', 'y', 'z'])
        csv_writer.writerows(results)

    # # Write the results to the output CSV file
    # with open(output_file, 'w', newline='') as csv_file:
    #     csv_writer = csv.writer(csv_file)
    #     csv_writer.writerow(['pcd_id', 'x', 'y', 'z'])
    #     for pcd_id, (x, y, z) in results.items():
    #         csv_writer.writerow([pcd_id, x, y, z])
        

# Example usage:
# input_csv_file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/filtered_points.csv'  # Replace with your input CSV file
# output_csv_file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/clustered_points.csv'
# output_csv_file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/clustered_points2.csv'
input_csv_file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/filtered_points_new.csv'  # Replace with your input CSV file
output_csv_file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/clustered_points_new.csv'
# output_csv_file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/viz_pointcloud_bin/clustered_points_new2.csv'
eps = 1.0  # Maximum distance for DBSCAN
min_samples = 2  # Minimum samples in a cluster
process_csv(input_csv_file, output_csv_file, eps, min_samples)
