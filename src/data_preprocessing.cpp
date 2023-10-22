//
// Created by user on 16/8/2023.
//
#include "data_preprocessing.h"
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream> // For stringstream
#include <vector>
#include <string>

std::vector<LidarPoint> readLidarData(const std::string& filePath) {
    std::vector<LidarPoint> lidarData;

    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return lidarData;
    }
    std::string line;
    std::getline(file, line); // Read and discard the header line

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double easting, northing;
        char comma;
        if (iss >> easting >> comma >> northing) {
            lidarData.push_back({easting, northing});
        }
    }
    return lidarData;
}

LidarPoint findClosestPoint(const std::vector<LidarPoint>& lidarData, double easting, double northing) {
    double minDistance = std::numeric_limits<double>::max();
    LidarPoint closestPoint;

    for (const LidarPoint& point : lidarData) {
        double distance = std::sqrt(std::pow(point.easting - easting, 2) + std::pow(point.northing - northing, 2));
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = point;
        }
    }

    return closestPoint;
}

double findClosestOffset(const std::vector<LidarPoint>& lidarData, double easting, double northing) {
    double minDistance = std::numeric_limits<double>::max();

    for (const LidarPoint& point : lidarData) {
        double distance = std::sqrt(std::pow(point.easting - easting, 2) + std::pow(point.northing - northing, 2));
        if (distance < minDistance) {
            minDistance = distance;
        }
    }

    return minDistance;
}

std::pair<double, double> findOffset(const std::vector<LidarPoint>& lidarData, double easting, double northing) {
    double minDistance_e = std::numeric_limits<double>::max();
    double minDistance_n = std::numeric_limits<double>::max();
    double minDistance = std::numeric_limits<double>::max();

//    double closestEasting = 0.0;
//    double closestNorthing = 0.0;

    for (const LidarPoint& point : lidarData) {
        double distance_e = std::sqrt(std::pow(point.easting - easting, 2));
        double distance_n = std::sqrt(std::pow(point.northing - northing, 2));
        double distance = std::sqrt(std::pow(point.easting - easting, 2) + std::pow(point.northing - northing, 2));
        if (distance < minDistance) {
            minDistance = distance;
            minDistance_e = distance_e;
            minDistance_n = distance_n;
        }
    }

//    for (const LidarPoint& point : lidarData) {
//        double distance = std::sqrt(std::pow(point.easting - easting, 2) + std::pow(point.northing - northing, 2));
//        if (distance < minDistance) {
//            minDistance = distance;
//            closestEasting = point.easting;
//            closestNorthing = point.northing;
//        }
//    }

    return std::make_pair(minDistance_e, minDistance_n);
//    return std::make_pair(closestEasting, closestNorthing);
}