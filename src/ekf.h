#ifndef EKF_HPP
#define EKF_HPP
using namespace std;
#include <vector>
#pragma once

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter() = default;
    void initialize(const double x[3], double xy_obs_noise_std, double yaw_rate_noise_std, double forward_velocity_noise_std, double initial_yaw_std, double lidar_xy);
    void update_gps(const double z[2]);
    void propagate(const double u[2], double dt);
    void update_lidar(const double l_actual[2], const double xlidar, const double ylidar);
//    void update_lidar(const double l_gt[2], const double l_obs[2], const double xlidar, const double ylidar);
//    void update_lidar2(const double l_pred[2], const double xlidar, const double ylidar);

    std::vector<std::pair<double, double>> landmark_positions_;
    double x_[3];
    double P_[9];
    double P[9];
    // Measurement error covariance Q
    double Q[4];
    double Ql[4];
    // state transition noise covariance R
    double R[9];
    double R_n[4];
};
#endif // EKF_HPP