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
#include "data_preprocessing.h"

int main() {
    // Create an instance of utmconv
    utmconv::utm_coords coords{};
    int counter = 0;
    std::vector<std::array<double, 3>> lidar_trajectory_xy;
    std::vector<std::array<double, 3>> obs_trajectory_xyz;
    std::vector<std::array<double, 3>> gt_trajectory_xyz;
    std::vector<double> gt_yaws; // [yaw_angle(rad),] x N
    std::vector<double> obs_yaw_rates; // [vehicle_yaw_rate(rad/s),] x N
    std::vector<double> obs_forward_velocities; // [vehicle_forward_velocity(m/s),] x N
    std::vector<double> ts;
    double last_non_empty_speed = 0.0;
    double last_non_empty_yaw = 0.0;
    double last_non_empty_yaw_rate = 0.0;
    double last_non_empty_x = 0.0;
    double last_non_empty_y = 0.0;

    double lidar_obs_easting = std::numeric_limits<double>::quiet_NaN();
    double lidar_obs_northing = std::numeric_limits<double>::quiet_NaN();
    double lidar_easting = std::numeric_limits<double>::quiet_NaN();
    double lidar_northing = std::numeric_limits<double>::quiet_NaN();

    double smooth_counter = 0.0;
    double kolpo = 0.0;

    std::vector<LidarPoint> lidarData = readLidarData("D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/lidar_gt_rtk_utm2.csv");
//    std::vector<LidarPoint> lidarData = readLidarData("D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/cmake-build-debug/lidar_gt_qgis_utm.csv");
    std::vector<std::array<double, 3>> lidar_x_y_z;
    double maxOffset = 0.0; // Initialize max offset to a small value or 0
    double t_maximum = 0.0;
    std::vector<std::array<double, 3>> lidar_obs_e_n;
    std::vector<std::array<double, 3>> lidar_gt_e_n;
    std::vector<double> distance_easting;
    std::vector<double> distance_northing;
    std::vector<double> easting_from_lidar;
    std::vector<double> northing_from_lidar;
    std::vector<double> times;

    std::ifstream raw_data("reordered_final_merge_new.csv");
//    std::ifstream raw_data("reordered_final_merge_2_ekfs.csv");
    std::string line, value;

// Read CSV file and store values in vectors
    std::getline(raw_data, line); // Skip header line

    long long int first_timestamp = 0;
    bool first_line = true;

    while (std::getline(raw_data, line)) {
        std::stringstream line_stream(line);
        std::vector<std::string> string_row;
        std::vector<double> row;

        std::getline(line_stream, value, ',');
        long long int timestamp = std::stoll(value);

        if (first_line) {
            first_timestamp = timestamp;
            first_line = false;
        }
        double elapsed_time = (timestamp - first_timestamp) / 1000000.0;
        ts.push_back(elapsed_time); // Store the elapsed time in seconds
        times.push_back(timestamp);
        while (std::getline(line_stream, value, ',')) {
            string_row.push_back(value);
            if (!value.empty()) {
                row.push_back(std::stold(value));
            }
            else {
                row.push_back(std::numeric_limits<double>::quiet_NaN()); // Push NaN to the row for empty value
            }
        }
        if (!string_row[0].empty()) { // WE HAVE GPS DATA
            last_non_empty_x = row[0];
            last_non_empty_y = row[1];
            obs_trajectory_xyz.push_back({row[0],row[1]});
            gt_trajectory_xyz.push_back({row[0],row[1]});
            last_non_empty_speed = row[3];
            obs_forward_velocities.push_back(last_non_empty_speed);
            gt_yaws.push_back(deg2rad(last_non_empty_yaw));
//            last_non_empty_yaw = row[16];
//            gt_yaws.push_back(row[16]); // GIA TO SISTIMA 2 EKF GIA SWSTO YAW
            obs_yaw_rates.push_back(last_non_empty_yaw_rate);
            lidar_x_y_z.push_back({row[13],row[14]}); //push nan nan
        } else if (!string_row[13].empty()) { // WE HAVE LIDAR DATA BUT NO GPS AND IMU DATA
            lidar_x_y_z.push_back({row[13],row[14]});
            gt_trajectory_xyz.push_back({last_non_empty_x,last_non_empty_y});
            obs_trajectory_xyz.push_back({row[0], row[1]}); //push nan nan
            obs_forward_velocities.push_back(last_non_empty_speed);
            gt_yaws.push_back(deg2rad(last_non_empty_yaw));
//            gt_yaws.push_back(last_non_empty_yaw); // GIA TO SISTIMA 2 EKF GIA SWSTO YAW
            obs_yaw_rates.push_back(last_non_empty_yaw_rate);
        } else { // WE DON'T HAVE GPS AND LIDAR DATA BUT WE HAVE IMU DATA
            gt_trajectory_xyz.push_back({last_non_empty_x,last_non_empty_y});
            obs_trajectory_xyz.push_back({row[0], row[1]}); //push nan nan
            obs_forward_velocities.push_back(last_non_empty_speed);
            last_non_empty_yaw_rate = row[12];
            last_non_empty_yaw = row[9];
            gt_yaws.push_back(deg2rad(row[9]));
//            last_non_empty_yaw = row[16];
//            gt_yaws.push_back(row[16]); // GIA TO SISTIMA 2 EKF GIA SWSTO YAW
            obs_yaw_rates.push_back(last_non_empty_yaw_rate);
            lidar_x_y_z.push_back({row[13],row[14]}); //push nan nan
        }
}
    size_t N = ts.size(); // Number of data points
    double xy_obs_noise_std = 30.0; // Standard deviation of observation noise of x and y in meters
    double yaw_rate_noise_std = 0.002; // Standard deviation of yaw rate in rad/s
    double forward_velocity_noise_std = 0.03; // Standard deviation of forward velocity in m/s
    double lidar_xy = 35.0;

    // Prepare initial estimate and its error covariance
    double initial_yaw_std = M_PI;
//    double initial_yaw = gt_yaws[0] + sample_normal_distribution(0, initial_yaw_std);
    double initial_yaw = gt_yaws[1];
    double x[3];
    x[0] = obs_trajectory_xyz[0][0];
    x[1] = obs_trajectory_xyz[0][1];
    x[2] = initial_yaw;
    // Initialize Kalman filter
    ExtendedKalmanFilter kf;
    kf.initialize(x, xy_obs_noise_std, yaw_rate_noise_std, forward_velocity_noise_std, initial_yaw_std,lidar_xy);

    std::vector<double> mu_x = {x[0]};
    std::vector<double> mu_y = {x[1]};
    std::vector<double> mu_theta = {x[2]};
    std::vector<double> var_x = {kf.P[0]};
    std::vector<double> var_y = {kf.P[4]};
    std::vector<double> var_theta = {kf.P[8]};

//
    double t_last = 0.0;
    for (size_t t_idx = 1; t_idx < N; ++t_idx) {
        double t = ts[t_idx];
        double dt = t - t_last;
        // Get control input `u = [v, omega] + noise`
        double u[2];
        u[0] = obs_forward_velocities[t_idx];
        u[1] = obs_yaw_rates[t_idx];
        // Because velocity and yaw rate are multiplied with `dt` in the state transition function,
        // Propagate!
        kf.propagate(u, dt);

        // Get measurement `z = [x, y] + noise`
        if (!std::isnan(obs_trajectory_xyz[t_idx][0])) {
            double z[2];
            z[0] = obs_trajectory_xyz[t_idx][0];
            z[1] = obs_trajectory_xyz[t_idx][1];
            // Update!
//            cout<<"yaw before the update of lidar: "<< normalize_angles(kf.x_[2]) << endl;
//            kf.update_gps(z);
//            cout<<"yaw after the update of lidar: "<< normalize_angles(kf.x_[2]) << endl;
            if (t_idx - smooth_counter > 12){
                // Update!
                kf.update_gps(z);
            }
        }
        if (std::isnan(lidar_x_y_z[t_idx][0])) {
            lidar_obs_e_n.push_back({std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()});
            lidar_gt_e_n.push_back({std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()});
            distance_easting.push_back(std::numeric_limits<double>::quiet_NaN());
            distance_northing.push_back(std::numeric_limits<double>::quiet_NaN());
            easting_from_lidar.push_back(std::numeric_limits<double>::quiet_NaN());
            northing_from_lidar.push_back(std::numeric_limits<double>::quiet_NaN());
        }
        if (!std::isnan(lidar_x_y_z[t_idx][0])) {
            smooth_counter = t_idx;
            // Compute the transformed values
            double imu_x1 = (lidar_x_y_z[t_idx][0] * std::cos(-M_PI/2)) - (lidar_x_y_z[t_idx][1] * std::sin(-M_PI/2)); //=y
            double imu_y1 = (lidar_x_y_z[t_idx][0] * std::sin(-M_PI/2)) + (lidar_x_y_z[t_idx][1] * std::cos(-M_PI/2)); //=-x
            // Compute using the latest values
            lidar_obs_easting = kf.x_[0] + ((imu_x1 * std::cos(normalize_angles(kf.x_[2]))) - (imu_y1 * std::sin(normalize_angles(kf.x_[2]))));
            lidar_obs_northing = kf.x_[1] + ((imu_x1 * std::sin(normalize_angles(kf.x_[2]))) + (imu_y1 * std::cos(normalize_angles(kf.x_[2]))));
//            lidar_obs_easting = kf.x_[0] + ((lidar_x_y_z[t_idx][1] * std::cos(gt_yaws[t_idx])) - (-lidar_x_y_z[t_idx][0] * std::sin(gt_yaws[t_idx])));
//            lidar_obs_northing = kf.x_[1] + ((lidar_x_y_z[t_idx][1] * std::sin(gt_yaws[t_idx])) + (-lidar_x_y_z[t_idx][0] * std::cos(gt_yaws[t_idx])));

//            lidar_obs_easting = kf.x_[0] + (lidar_x_y_z[t_idx][0] * std::sin(gt_yaws[t_idx])) + (lidar_x_y_z[t_idx][1] * std::cos(gt_yaws[t_idx]));
//            lidar_obs_northing = kf.x_[1] - (lidar_x_y_z[t_idx][0] * std::cos(gt_yaws[t_idx])) + (lidar_x_y_z[t_idx][1] * std::sin(gt_yaws[t_idx]));
//            lidar_obs_easting = kf.x_[0] + (lidar_x_y_z[t_idx][0] * std::sin(normalize_angles(kf.x_[2]))) + (lidar_x_y_z[t_idx][1] * std::cos(normalize_angles(kf.x_[2])));
//            lidar_obs_northing = kf.x_[1] - (lidar_x_y_z[t_idx][0] * std::cos(normalize_angles(kf.x_[2]))) + (lidar_x_y_z[t_idx][1] * std::sin(normalize_angles(kf.x_[2])));
//            lidar_obs_easting = gt_trajectory_xyz[t_idx][0] + (lidar_x_y_z[t_idx][0] * std::sin(normalize_angles(kf.x_[2]))) + (lidar_x_y_z[t_idx][1] * std::cos(normalize_angles(kf.x_[2])));
//            lidar_obs_northing = gt_trajectory_xyz[t_idx][1] - (lidar_x_y_z[t_idx][0] * std::cos(normalize_angles(kf.x_[2]))) + (lidar_x_y_z[t_idx][1] * std::sin(normalize_angles(kf.x_[2])));
//             Store Lidar observation easting and northing
            lidar_obs_e_n.push_back({lidar_obs_easting, lidar_obs_northing});
            LidarPoint closestPoint = findClosestPoint(lidarData, lidar_obs_easting, lidar_obs_northing);
            lidar_gt_e_n.push_back({closestPoint.easting, closestPoint.northing});
//            cout<<"yaw_pred "<<kf.x_[2]<<endl;
//            cout<<"de "<<closestPoint.easting - kf.x_[0]<<endl;
//            cout<<"dn "<<closestPoint.northing - kf.x_[1] <<endl;
//            cout<<"theta "<< atan2(closestPoint.northing,closestPoint.easting) << endl;
//            cout <<"theta2 "<<atan2(lidar_x_y_z[t_idx][0], lidar_x_y_z[t_idx][1])<<endl;
//            cout<<"yaw actual "<<-atan2(closestPoint.northing,closestPoint.easting) - atan2(lidar_x_y_z[t_idx][1], lidar_x_y_z[t_idx][0])<<endl;
//            cout<<"t_idx "<<t_idx<<endl;
//            cout<< fixed << setprecision(5) << "kf_x "<<kf.x_[0]<<endl;
//            cout<< fixed << setprecision(5) << "kf_y "<<kf.x_[1]<<endl;
//            cout<< fixed << setprecision(5) << "x "<<lidar_x_y_z[t_idx][0]<<endl;
//            cout<< fixed << setprecision(5) << "y "<<lidar_x_y_z[t_idx][1]<<endl;
//            cout<< fixed << setprecision(5) << "yaw "<<normalize_angles(kf.x_[2])<<endl;
//            cout<< fixed << setprecision(5) << "obs_east "<<lidar_obs_easting<<endl;
//            cout<< fixed << setprecision(5) << "obs_north "<<lidar_obs_northing<<endl;
//            cout<< fixed << setprecision(5) << "closest "<<closestPoint.easting<<endl;

            // Calculate lidar_easting and lidar_northing using last_valid_yaw
            lidar_easting = closestPoint.easting + ((-imu_x1 * std::cos(normalize_angles(kf.x_[2]))) + (imu_y1 * std::sin(normalize_angles(kf.x_[2]))));
            lidar_northing = closestPoint.northing + ((-imu_x1 * std::sin(normalize_angles(kf.x_[2]))) - (imu_y1 * std::cos(normalize_angles(kf.x_[2]))));
//            lidar_easting = closestPoint.easting + ((-lidar_x_y_z[t_idx][1] * std::cos(gt_yaws[t_idx])) - (lidar_x_y_z[t_idx][0] * std::sin(gt_yaws[t_idx])));
//            lidar_northing = closestPoint.northing + ((-lidar_x_y_z[t_idx][1] * std::sin(gt_yaws[t_idx])) + (lidar_x_y_z[t_idx][0] * std::cos(gt_yaws[t_idx])));

//            lidar_easting = closestPoint.easting - (lidar_x_y_z[t_idx][0] * std::sin(gt_yaws[t_idx])) - (lidar_x_y_z[t_idx][1] * std::cos(gt_yaws[t_idx]));
//            lidar_northing = closestPoint.northing + (lidar_x_y_z[t_idx][0] * std::cos(gt_yaws[t_idx])) - (lidar_x_y_z[t_idx][1] * std::sin(gt_yaws[t_idx]));
//            lidar_easting = closestPoint.easting - (lidar_x_y_z[t_idx][0] * std::sin(normalize_angles(kf.x_[2]))) - (lidar_x_y_z[t_idx][1] * std::cos(normalize_angles(kf.x_[2])));
//            lidar_northing = closestPoint.northing + (lidar_x_y_z[t_idx][0] * std::cos(normalize_angles(kf.x_[2]))) - (lidar_x_y_z[t_idx][1] * std::sin(normalize_angles(kf.x_[2])));
            easting_from_lidar.push_back(lidar_easting);
            northing_from_lidar.push_back(lidar_northing);

            double distance = findClosestOffset(lidarData, lidar_obs_easting, lidar_obs_northing);
            std::pair<double, double> closestOffset = findOffset(lidarData, lidar_obs_easting, lidar_obs_northing);
            double closestEasting = closestOffset.first;
            double closestNorthing = closestOffset.second;
            distance_easting.push_back(closestEasting);
            distance_northing.push_back(closestNorthing);

            if (distance > maxOffset) {
                maxOffset = distance;
                t_maximum = t_idx + 1;
            }
            if (distance > 5 and t_idx < 12000) {
                cout << "time with big offset: " << t_idx + 1 << " with distance: " << distance << " and state_angle: "
                << normalize_angles(kf.x_[2]) << " and angle: " << kf.x_[2] <<endl;
                counter = counter + 1;
            }
            double l[2];
            double l_obs[2];
            double l_gt[2];
            l_gt[0] = closestPoint.easting;
            l_gt[1] = closestPoint.northing;
            l[0] = lidar_easting;
            l[1] = lidar_northing;
            l_obs[0] = lidar_obs_easting;
            l_obs[1] = lidar_obs_northing;
            // Update!
//            cout<<"yaw before the update of lidar: "<< normalize_angles(kf.x_[2]) << endl;
            kf.update_lidar(l, lidar_x_y_z[t_idx][0], lidar_x_y_z[t_idx][1]);
//            cout<<"yaw after the update of lidar: "<< normalize_angles(kf.x_[2]) << endl;
//            kf.update_lidar(l_gt, l_obs, lidar_x_y_z[t_idx][0], lidar_x_y_z[t_idx][1]);
        }
        // Save estimated state to analyze later
        mu_x.push_back(kf.x_[0]);
        mu_y.push_back(kf.x_[1]);
        mu_theta.push_back(normalize_angles(kf.x_[2]));
        // Save estimated variance to analyze later
        var_x.push_back(kf.P_[0]);
        var_y.push_back(kf.P_[4]);
        var_theta.push_back(kf.P_[8]);
        t_last = t;
    }
//    // Print the maximum offset after the loop completes
//    cout << "Maximum Offset: " << maxOffset << " in timestamp: " << t_maximum <<endl;
    cout << "counter of points more than 5.0 offset : " << counter << endl; //1541 / 3157

    // mu_x, mu_y, and mu_theta are the estimated 2D pose [x, y, theta]
    // var_x, var_y, and var_theta are the estimated error variances of 2D pose
//    std::ofstream output_file("output_reordered_final_merge_2_ekfs.csv");
    std::ofstream output_file("output_reordered_final_merge_new.csv");

    output_file << "easting,northing,yaw,state_x,state_y,state_yaw,obs_e,obs_n,gt_e,gt_n,distance_east,distance_north,lidar_easting,lidar_northing" << std::endl;
    for (size_t i = 0; i < obs_trajectory_xyz.size(); ++i) {
//        double time = times[i];
        double lon = gt_trajectory_xyz[i][0];
        double lat = gt_trajectory_xyz[i][1];
        double yaw = gt_yaws[i];
        double state_x = mu_x[i];
        double state_y = mu_y[i];
        double state_yaw = mu_theta[i];
        double lidar_obs_e = lidar_obs_e_n[i][0];
        double lidar_obs_n = lidar_obs_e_n[i][1];
        double closestPoint_e = lidar_gt_e_n[i][0];
        double closestPoint_n = lidar_gt_e_n[i][1];
        double distance_east = distance_easting[i];
        double distance_north = distance_northing[i];
        double east_from_lidar = easting_from_lidar[i];
        double north_from_lidar = northing_from_lidar[i];

//        output_file << fixed << std::setprecision(1) << time << ",";
        output_file << std::setprecision(14)
                    << lon << "," << lat << "," << yaw << ","
                    << state_x << "," << state_y << "," << state_yaw <<  ","
                    << lidar_obs_e << "," << lidar_obs_n << "," << closestPoint_e
                    << "," << closestPoint_n << "," << distance_east << ","
                    << distance_north << "," << east_from_lidar << "," << north_from_lidar
                    << std::endl;
    }
    output_file.close();
    return 0;
}
