//
// Created by user on 16/8/2023.
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
    std::vector<std::array<double, 3>> lidar_gt;

//    std::ifstream raw_data("lidar_gt_gearth_geodetic.csv");
//    std::ifstream raw_data("lidar_gt_gmap_geodetic.csv");
//    std::ifstream raw_data("lidar_gt_rtk_geodetic.csv");
    std::ifstream raw_data("lidar_gt_rtk_geodetic2.csv");
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
        utmconv::geodetic_to_utm(row[0], row[1], coords);
        lidar_gt.push_back({coords.easting, coords.northing});
    }

//    std::ofstream output_file("lidar_gt_gearth_utm.csv");
//    std::ofstream output_file("lidar_gt_gmap_utm.csv");
//    std::ofstream output_file("lidar_gt_rtk_utm.csv");
    std::ofstream output_file("lidar_gt_rtk_utm2.csv");
    output_file << "easting,northing " << std::endl;
    for (size_t i = 0; i < lidar_gt.size(); ++i) {
        double easting = lidar_gt[i][0];
        double northing = lidar_gt[i][1];

        output_file << std::fixed << easting << "," << northing << std::endl;
    }
    output_file.close();
    return 0;
}