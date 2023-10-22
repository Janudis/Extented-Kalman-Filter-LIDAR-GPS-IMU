//
// Created by user on 24/8/2023.
//
#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include "lidar_preprocessing.h"

std::tuple<std::vector<Point>, std::vector<double>, std::vector<double>, size_t> load_pointcloud_mine(const std::vector<Point>& points) {
    std::vector<double> intensity(points.size());
    std::vector<Point> pointcloud(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        intensity[i] = points[i][3];
        pointcloud[i] = {points[i][0], points[i][1], points[i][2]};
    }

    std::vector<Point> filtered_points;
    std::vector<double> filtered_intensity;
    std::vector<double> distance;
    size_t num_points = 0;

//first filtering: intensity>0.8
    for (size_t i = 0; i < points.size(); ++i) {
        if (intensity[i] > 0.8) {
            double distance_i = std::sqrt(pointcloud[i][0] * pointcloud[i][0] +
                                          pointcloud[i][1] * pointcloud[i][1] +
                                          pointcloud[i][2] * pointcloud[i][2]);
// second filtering: distance<50 kai z>0 (panw apo to lidar)
            if (distance_i < 50 && pointcloud[i][2] > 0) {
                filtered_points.push_back(pointcloud[i]);
                filtered_intensity.push_back(intensity[i]);
                distance.push_back(distance_i);
                ++num_points;
            }
        }
    }
//return only when there are more than 10 filtered lidar points in this timestamp
    if (num_points > 10) {
        return std::make_tuple(filtered_points, filtered_intensity, distance, num_points);
    } else {
        return std::make_tuple(std::vector<Point>(), std::vector<double>(), std::vector<double>(), 0);
    }
}

//int main() {
//    // Example usage
//    std::vector<Point> lidar_data = {
//            {0.03992553, 0.77477145, -0.36176127, 0.03921569},
//            {0.12345678, 0.23456789, 0.34567890, 0.9},
//            {-0.12345678, -0.23456789, -0.34567890, 0.7},
//            {0.53992553, 0.17477145, -0.12176127, 0.8},
//            {0.22345678, 0.43456789, 0.54567890, 0.6},
//            {0.01345678, 0.03456789, -0.04567890, 0.3},
//            {-0.42345678, -0.73456789, 0.94567890, 0.1},
//            {-0.32345678, 0.03456789, -0.74567890, 0.5},
//            {0.52345678, -0.53456789, 0.14567890, 0.2},
//            {0.13992553, 0.37477145, -0.25176127, 0.7},
//            {0.72345678, 0.83456789, -0.54567890, 0.4},
//            {0.42345678, 0.23456789, 0.94567890, 0.9},
//            {-0.52345678, 0.63456789, -0.14567890, 0.6},
//            {-0.82345678, 0.03456789, 0.74567890, 0.3},
//            {0.12345678, -0.73456789, -0.84567890, 0.2},
//            {0.82345678, 0.13456789, 0.04567890, 0.1},
//            {0.03992553, -0.27477145, -0.36176127, 0.7},
//            {-0.72345678, 0.13456789, -0.54567890, 0.8},
//            {-0.52345678, 0.43456789, 0.74567890, 0.5},
//            {0.62345678, 0.03456789, -0.14567890, 0.4}
//    };
//
//    auto result = load_pointcloud_mine(lidar_data);
//
//    if (std::get<3>(result) > 10) {
//        // Process the filtered points when there are more than 10
//        std::vector<Point> filtered_points = std::get<0>(result);
//        std::vector<double> filtered_intensity = std::get<1>(result);
//        // ... other processing ...
//    } else {
//        std::cout << "There are less than 10 filtered points." << std::endl;
//    }
//
//    return 0;
//}