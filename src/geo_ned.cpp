#include "geo_ned.h"
#include "matrix_operations.h"
#include <iostream>
//using namespace Eigen;
using namespace std;

double a = 6378137.0;
double f = 1.0 / 298.257223563;
double b = (1.0 - f) * a;
double e = sqrt(a * a - b * b) / a;

double deg2rad(double deg){
    return deg *M_PI / 180.0;
}

double sample_normal_distribution(double mean, double std_dev) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> distribution(mean, std_dev);
    return distribution(gen);
}

double normalize_angles(double angle) {
    angle = (std::fmod(angle + M_PI, 2.0 * M_PI));
    if (angle < 0) angle += (2.0 * M_PI);
    angle -= M_PI;
    return angle;
}


void Rx(double theta, double mat[9]) {
    mat[0] = 1;
    mat[1] = 0;
    mat[2] = 0;
    mat[3] = 0;
    mat[4] = std::cos(theta);
    mat[5] = -std::sin(theta);
    mat[6] = 0;
    mat[7] = std::sin(theta);
    mat[8] = std::cos(theta);
}

void Ry(double theta, double mat[9]) {
    mat[0] = std::cos(theta);
    mat[1] = 0;
    mat[2] = std::sin(theta);
    mat[3] = 0;
    mat[4] = 1;
    mat[5] = 0;
    mat[6] = -std::sin(theta);
    mat[7] = 0;
    mat[8] = std::cos(theta);
}

void Rz(double theta, double mat[9]) {
    mat[0] = std::cos(theta);
    mat[1] = -std::sin(theta);
    mat[2] = 0;
    mat[3] = std::sin(theta);
    mat[4] = std::cos(theta);
    mat[5] = 0;
    mat[6] = 0;
    mat[7] = 0;
    mat[8] = 1;
}

//std::vector<std::array<double, 3>> lla_to_ecef(const std::vector<std::array<double, 3>>& points_lla) {
//    std::vector<std::array<double, 3>> points_ecef(points_lla.size());
//
//    for (int i = 0; i < points_lla.size(); i++) {
//        double lon = points_lla[i][0] * M_PI / 180.0;
//        double lat = points_lla[i][1] * M_PI / 180.0;
//        double alt = points_lla[i][2];
//        double N = a / sqrt(1.0 - (e * sin(lat)) * (e * sin(lat)));
//        double x = (N + alt) * cos(lat) * cos(lon);
//        double y = (N + alt) * cos(lat) * sin(lon);
//        double z = (N * (1.0 - e * e) + alt) * sin(lat);
//        points_ecef[i] = { x, y, z };
//    }
//    return points_ecef;
//}
//
//
//std::vector<std::array<double, 3>> ecef_to_enu(const std::vector<std::array<double, 3>>& points_ecef, const std::array<double, 3>& ref_lla) {
//    std::vector<std::array<double, 3>> points_enu(points_ecef.size());
//
//    double lon = deg2rad(ref_lla[0]);
//    double lat = deg2rad(ref_lla[1]);
//    double alt = ref_lla[2];
//
//    std::array<double, 3> ref_ecef = lla_to_ecef({ref_lla})[0];
//
//    double Rz1[9], Ry1[9], Rz2[9], R[9], temp[9];
//    Rz(M_PI / 2.0, Rz1);
//    Ry(M_PI / 2.0 - lat, Ry1);
//    Rz(lon, Rz2);
//    matmul3x3(temp, Rz1, Ry1);
//    matmul3x3(R, temp, Rz2);
//
//    for (int i = 0; i < points_ecef.size(); i++) {
//        double relative[3] = {points_ecef[i][0] - ref_ecef[0], points_ecef[i][1] - ref_ecef[1],
//                              points_ecef[i][2] - ref_ecef[2]};
//        double enu[3];
//        matvecmul3x3(enu, R, relative);
//        points_enu[i] = {enu[0], enu[1], enu[2]};
//
//    }
//    return points_enu;
//}
//
//std::vector<std::array<double, 3>> lla_to_enu(const std::vector<std::array<double, 3>>& points_lla, const std::array<double, 3>& ref_lla) {
//    std::vector<std::array<double, 3>> points_ecef = lla_to_ecef(points_lla);
//    return ecef_to_enu(points_ecef, ref_lla);
//}