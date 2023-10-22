//
// Created by user on 3/8/2023.
//
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "ekf.h"
#include "geo_ned.h"
#include "utm.h"
#include "matrix_operations.h"
using namespace std;
#include <iomanip>

int main() {
    utmconv::utm_coords coords{};
    std::vector<std::array<double, 3>> gps_data;
    std::vector<double> time;
    std::vector<double> alt;
    std::vector<double> speed;
//    std::vector<std::array<double, 3>> lidar_gt;
//    std::ifstream raw_data("gps_data.csv");
//    std::ifstream raw_data("D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/code_to_read_data/gps_read_data/gps_data_new.csv");
    std::ifstream raw_data("D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/localization_log_gps_34.csv");
    std::string line, value;

    // Read CSV file and store values in vectors
    std::getline(raw_data, line); // Skip header line

    while (std::getline(raw_data, line)) {
        std::stringstream line_stream(line);
        std::vector<double> row;

        while (std::getline(line_stream, value, ',')) {
            if (!value.empty()) {
                row.push_back(std::stold(value));
            }
            else {
                row.push_back(std::numeric_limits<double>::quiet_NaN()); // Push NaN to the row for empty value
            }
        }

        utmconv::geodetic_to_utm(row[1], row[2], coords);
        gps_data.push_back({coords.northing, coords.easting});
//        utmconv::geodetic_to_utm(row[0], row[1], coords);
//        lidar_gt.push_back({coords.easting, coords.northing});
        time.push_back(row[0]);
        alt.push_back(row[3]);
        speed.push_back(row[4]);
    }

//    std::ofstream output_file("output_gps_data_new.csv");
    std::ofstream output_file("localization_log_gps_34_utm.csv");
//    output_file << std::setprecision(12) << "timestamp,easting,northing,altitude,speed" << std::endl;
     output_file << "timestamp,easting,northing,altitude,speed" << std::endl;
    for (size_t i = 0; i < gps_data.size(); ++i) {
        double times = time[i];
        double utm_x = gps_data[i][1];
        double utm_y = gps_data[i][0];
        double alts = alt[i];
        double speeds = speed[i];

        output_file << std::fixed << times << "," << utm_x << "," << utm_y << "," << alts << "," << speeds << std::endl;
    }

//    std::ofstream output_file("lidar_gt_gmap_utm.csv");
//    output_file << "easting,northing " << std::endl;
//    for (size_t i = 0; i < lidar_gt.size(); ++i) {
//        double easting = lidar_gt[i][0];
//        double northing = lidar_gt[i][1];
//
//        output_file << std::fixed << easting << "," << northing << std::endl;
//    }

    output_file.close();
    return 0;
}