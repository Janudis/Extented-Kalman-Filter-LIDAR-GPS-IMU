//
// Created by user on 24/8/2023.
//

#ifndef FUSION_CAMERA_GPS_IMU_LIDAR_PREPROCESSING_H
#define FUSION_CAMERA_GPS_IMU_LIDAR_PREPROCESSING_H

#include <vector>
#include <tuple>

using Point = std::vector<double>;

std::tuple<std::vector<Point>, std::vector<double>, std::vector<double>, size_t> load_pointcloud_mine(const std::vector<Point>& points);

#endif //FUSION_CAMERA_GPS_IMU_LIDAR_PREPROCESSING_H
