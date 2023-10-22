//
// Created by user on 24/8/2023.
//

#ifndef FUSION_CAMERA_GPS_IMU_DBSCAN_CLUSTERING_H
#define FUSION_CAMERA_GPS_IMU_DBSCAN_CLUSTERING_H

#include <vector>

struct Point {
    double x, y, z;
    Point() : x(0), y(0), z(0) {}
    Point(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

std::vector<int> dbscan_clustering(const std::vector<Point>& points, double eps, int min_samples);

std::vector<std::pair<Point, int>> points_with_clusters(const std::vector<Point>& points, const std::vector<int>& cluster_labels);

Point find_closest_cluster_centroid(const std::vector<std::pair<Point, int>>& clustered_points);

#endif //FUSION_CAMERA_GPS_IMU_DBSCAN_CLUSTERING_H
