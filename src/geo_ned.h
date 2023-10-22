#ifndef GEO_NED_HPP
#define GEO_NED_HPP

#include <cmath>
#include <vector>
#include <array>
//#include <Eigen/Dense>
#include <random>

double normalize_angles(double angle);
double sample_normal_distribution(double mean, double stddev);
double deg2rad(double deg);

void Rx(double theta, double mat[9]);
void Ry(double theta, double mat[9]);
void Rz(double theta, double mat[9]);

std::vector<std::array<double, 3>> lla_to_ecef(const std::vector<std::array<double, 3>>& points_lla) ;
std::vector<std::array<double, 3>> ecef_to_enu(const std::vector<std::array<double, 3>>& points_ecef, const std::array<double, 3>& ref_lla) ;
std::vector<std::array<double, 3>> lla_to_enu(const std::vector<std::array<double, 3>>& points_lla, const std::array<double, 3>& ref_lla);

#endif // GEO_NED_HPP
