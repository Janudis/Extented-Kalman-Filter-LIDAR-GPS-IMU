//
// Created by user on 24/8/2023.
//
#include <iostream>
#include <vector>
#include <cmath>
#include "DBSCAN_clustering.h"
#include <unordered_map>  // Add this include for unordered_map
#include <map>  // Add this include for unordered_map
#include <iomanip>

std::vector<int> dbscan_clustering(const std::vector<Point>& points, double eps, int min_samples) {
    int n = points.size();
    std::vector<int> cluster_labels(n, 0);
    int cluster_id = 1;

    for (int i = 0; i < n; ++i) {
        if (cluster_labels[i] != 0) {
            continue;
        }

        std::vector<int> neighbors;
        for (int j = 0; j < n; ++j) {
            double distance = std::sqrt(std::pow(points[j].x - points[i].x, 2) +
                                        std::pow(points[j].y - points[i].y, 2) +
                                        std::pow(points[j].z - points[i].z, 2));
            if (distance <= eps) {
                neighbors.push_back(j);
            }
        }

        if (neighbors.size() < min_samples) {
            cluster_labels[i] = -1;
        } else {
            cluster_labels[i] = cluster_id;

            for (size_t j = 0; j < neighbors.size(); ++j) {
                int current_point = neighbors[j];

                if (cluster_labels[current_point] == 0) {
                    cluster_labels[current_point] = cluster_id;
                    std::vector<int> new_neighbors;
                    for (int k = 0; k < n; ++k) {
                        double distance = std::sqrt(std::pow(points[k].x - points[current_point].x, 2) +
                                                    std::pow(points[k].y - points[current_point].y, 2) +
                                                    std::pow(points[k].z - points[current_point].z, 2));
                        if (distance <= eps) {
                            new_neighbors.push_back(k);
                        }
                    }

                    if (new_neighbors.size() >= min_samples) {
                        neighbors.insert(neighbors.end(), new_neighbors.begin(), new_neighbors.end());
                    }
                }
            }

            ++cluster_id;
        }
    }

    return cluster_labels;
}

std::vector<std::pair<Point, int>> points_with_clusters(const std::vector<Point>& points, const std::vector<int>& cluster_labels) {
    std::vector<std::pair<Point, int>> points_with_clusters_list;
    for (size_t i = 0; i < points.size(); ++i) {
        points_with_clusters_list.push_back(std::make_pair(points[i], cluster_labels[i]));
    }
    return points_with_clusters_list;
}

Point find_closest_cluster_centroid(const std::vector<std::pair<Point, int>>& clustered_points) {
    const double epsilon = 1e-6;
    Point reference_point(0, 0, 0);

    std::map<int, std::vector<Point>> cluster_centroids; // Use a map to store cluster centroids
    for (const auto& point_cluster : clustered_points) {
        if (point_cluster.second != -1) {
            cluster_centroids[point_cluster.second].push_back(point_cluster.first);
        }
    }

    double min_distance = std::numeric_limits<double>::max();
    Point closest_cluster;

    for (const auto& cluster_points : cluster_centroids) {
        const std::vector<Point>& points_in_cluster = cluster_points.second;

        double centroid_x = 0, centroid_y = 0, centroid_z = 0;
        for (const Point& point : points_in_cluster) {
            centroid_x += point.x;
            centroid_y += point.y;
            centroid_z += point.z;
        }

        centroid_x /= static_cast<double>(points_in_cluster.size());
        centroid_y /= static_cast<double>(points_in_cluster.size());
        centroid_z /= static_cast<double>(points_in_cluster.size());

        Point centroid(centroid_x, centroid_y, centroid_z);

        double distance = std::sqrt(std::pow(centroid.x - reference_point.x, 2) +
                                    std::pow(centroid.y - reference_point.y, 2) +
                                    std::pow(centroid.z - reference_point.z, 2));

        if (distance + epsilon < min_distance) {
            min_distance = distance;
            closest_cluster = centroid;
        }
    }

    return closest_cluster;
}

//int main() {
//    // Example usage
//    double eps = 1.0;
//    int min_samples = 10;
//    // Create the data points
//    std::vector<Point> filtered_points = {
//            {26.90315, 11.139115, 0.33898836},
//            {26.852713, 11.222729, 0.50800467},
//            {44.153698, 12.587465, 0.534512},
//            {44.193516, 12.432184, 0.53446543},
//            {44.238785, 12.2701235, 0.53446543},
//            {44.157646, 12.588653, 0.80148435},
//            {44.20132, 12.434447, 0.80148435},
//            {44.240597, 12.279008, 0.8014145},
//            {-1.8376133, -17.858814, 0.73142844},
//            {-1.9039044, -17.85991, 0.7317542},
//            {-2.091886, -17.875084, 0.7332196},
//            {-2.1592956, -17.883171, 0.733871},
//            {-2.2246685, -17.899317, 0.73484784},
//            {-2.2855275, -17.903736, 0.7353364},
//            {-2.3500936, -17.911499, 0.73598766},
//            {-2.4147289, -17.919031, 0.736639},
//            {-2.023413, -17.856863, 0.41817823},
//            {-2.0844717, -17.865942, 0.41855043},
//            {-2.1528034, -17.882004, 0.41910875},
//            {-2.2145472, -17.89461, 0.41957402},
//            {-2.2816365, -17.898272, 0.41985315},
//            {-1.8948466, -17.864182, 0.52281505},
//            {-1.9549507, -17.86575, 0.5230478},
//            {-2.018191, -17.866762, 0.52328056},
//            {-2.0819383, -17.871523, 0.5236296},
//            {-2.1488502, -17.875683, 0.5239787},
//            {-2.2064672, -17.880747, 0.52432775},
//            {-2.2708838, -17.888803, 0.52479327},
//            {-2.3369315, -17.908522, 0.52560776}
//    };
//
//    std::vector<int> cluster_labels = dbscan_clustering(filtered_points, eps, min_samples);
//    std::vector<std::pair<Point, int>> points_clustered = points_with_clusters(filtered_points, cluster_labels);
//    Point closest_cluster_centroid = find_closest_cluster_centroid(points_clustered);
//    std::cout << std::setprecision(9) << closest_cluster_centroid.x<<std::endl;
//    std::cout<< std::setprecision(8) << closest_cluster_centroid.y<<std::endl;
//    std::cout<< std::setprecision(8) << closest_cluster_centroid.z<<std::endl;
//
//    return 0;
//}
