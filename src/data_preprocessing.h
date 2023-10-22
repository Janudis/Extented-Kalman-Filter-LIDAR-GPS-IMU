//
// Created by user on 16/8/2023.
//

// data_preprocessing.h

#ifndef DATA_PREPROCESSING_H
#define DATA_PREPROCESSING_H

#include <vector>
#include <string>

// Define the LidarPoint struct
struct LidarPoint {
    double easting;
    double northing;
};

std::vector<LidarPoint> readLidarData(const std::string& filePath);
LidarPoint findClosestPoint(const std::vector<LidarPoint>& lidarData, double easting, double northing);
double findClosestOffset(const std::vector<LidarPoint>& lidarData, double easting, double northing);
std::pair<double, double> findOffset(const std::vector<LidarPoint>& lidarData, double easting, double northing);

#endif // DATA_PREPROCESSING_H


