#include "ekf.h"
#include <cmath> // for std::sin and std::cos
#include "geo_ned.h"
#include "matrix_operations.h"
using namespace std;
#include "iostream"
#include <iomanip>

void ExtendedKalmanFilter::initialize(const double x[3], double xy_obs_noise_std, double yaw_rate_noise_std, double forward_velocity_noise_std, double initial_yaw_std, double lidar_xy) {
    x_[0] = x[0];
    x_[1] = x[1];
    x_[2] = x[2];

    P[0] = 25.0;
    P[1] = 0;
    P[2] = 0;
    P[3] = 0;
    P[4] = 25.0;
    P[5] = 0;
    P[6] = 0;
    P[7] = 0;
    P[8] = initial_yaw_std * initial_yaw_std;

    P_[0] = P[0];
    P_[1] = P[1];
    P_[2] = P[2];
    P_[3] = P[3];
    P_[4] = P[4];
    P_[5] = P[5];
    P_[6] = P[6];
    P_[7] = P[7];
    P_[8] = P[8];

    Q[0] = xy_obs_noise_std * xy_obs_noise_std;
    Q[1] = 0;
    Q[2] = 0;
    Q[3] = xy_obs_noise_std * xy_obs_noise_std;

    Ql[0] = lidar_xy * lidar_xy;
    Ql[1] = 0;
    Ql[2] = 0;
    Ql[3] = lidar_xy * lidar_xy;

    R_n[0] = forward_velocity_noise_std * forward_velocity_noise_std; //acceleration noise
    R_n[1] = 0;
    R_n[2] = 0;
    R_n[3] = yaw_rate_noise_std * yaw_rate_noise_std;

    R[0] = forward_velocity_noise_std * forward_velocity_noise_std;
    R[1] = 0;
    R[2] = 0;
    R[3] = 0;
    R[4] = forward_velocity_noise_std * forward_velocity_noise_std;
    R[5] = 0;
    R[6] = 0;
    R[7] = 0;
    R[8] = yaw_rate_noise_std * yaw_rate_noise_std;
}

void ExtendedKalmanFilter::update_gps(const double z[2]) {
    double H[6] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0}; //2x3
    double I[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    //double HT[6] = {1.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    double z_[2] = {x_[0], x_[1]}; //predicted
    double HT[6], HP_[6], HPHt[4], HPHt_Q_inv[4], K[6], K_mult_z_diff[3], KH[9], KHP_[9],HPHt_Q[2],P_Ht[6], IK[9];

    transposeMatrix3x2(HT, H);
    Mat2x3_Mul_Mat3(HP_,H,P_);//H 2X3 P_ 3X3 -->2X3
    matmul2x3x3(HPHt,HP_,HT); //HP_ 2X3 HT 3x2-->2x2 //lathos
    addMatrix2x2(HPHt_Q, HPHt, Q);
    inverseMatrix2x2(HPHt_Q_inv, HPHt_Q);
    matmul3x3x2(P_Ht, P_, HT); //P_ 3X3, HT 3x2--->3x2
    matmul3x2x2(K, P_Ht, HPHt_Q_inv); //P_Ht 3X2, HPHt_Q_inv 2x2-->3X2

    double z_diff[2] = {z[0] - z_[0], z[1] - z_[1]};
    matvecmul3x2(K_mult_z_diff, K, z_diff); //K 3X2 z_diff 2X1 -->3X1
    addVectors3D(x_, x_, K_mult_z_diff);
    x_[2] = normalize_angles(x_[2]);
//    x_[0] += K_mult_z_diff[0];
//    x_[1] += K_mult_z_diff[1];

    Mat3_AxB(KH, K, H); //K 3X2 H 2X3-->3X3
//    matmul3x3(KHP_,KH,P_);  //KH 3X3 P_ 3X3 --> 3X3
//    subtractMatrix3x3(P_, P_, KHP_);

    subtractMatrix3x3(IK, I, KH);
    matmul3x3(P_, IK, P_);
//    for(int i=0; i<2; i++)
//        std::cout << "z_diff[" << i << "] = " << z_diff[i] << std::endl;
}

void ExtendedKalmanFilter::propagate(const double u[2], double dt) {
    double dt_squared = dt * dt; // Calculate dt^2
    double v = u[0];
    double omega = u[1];
    double theta = x_[2];
    double dtheta = omega * dt; //swsto
    double r = v / omega;
    double dx = -r * sin(theta) + r * sin(theta + dtheta);
    double dy = r * cos(theta) - r * cos(theta + dtheta);
//    double ve = cos( x_[2] ) * v;
//    double vn = sin( x_[2] ) * v;
//    double dx = ve * dt;
//    double dy = vn * dt;
    x_[0] += dx;
    x_[1] += dy;
    x_[2] += dtheta;
    x_[2] = normalize_angles(x_[2]);

    double G[9], GP_[9], G_P_Gt[9], GT[9];
    G[0] = 1.0;
    G[1] = 0.0;
    G[2] = -r * cos(theta) + r * cos(theta + dtheta);
    G[3] = 0.0;
    G[4] = 1.0;
    G[5] = -r * sin(theta) + r * sin(theta + dtheta);
    G[6] = 0.0;
    G[7] = 0.0;
    G[8] = 1.0;
//    G[0] = 1.0;
//    G[1] = 0.0;
//    G[2] = -sin(theta)*v*dt;
//    G[3] = 0.0;
//    G[4] = 1.0;
//    G[5] = cos(theta)*v*dt;
//    G[6] = 0.0;
//    G[7] = 0.0;
//    G[8] = 1.0;

    // Calculate process covariance matrix
    double G_n[6], G_n_T[6], G_nR_n[6];
    G_n[0] = 0.5*(dt*dt)*cos(theta);
    G_n[1] = 0.0;
    G_n[2] = 0.5*(dt*dt)*sin(theta);
    G_n[3] = 0.0;
    G_n[4] = 0.0;
    G_n[5] = 0.5*(dt*dt); //3x2

    G_nR_n[0] = G_n[0]*R_n[0];
    G_nR_n[1] = 0.0;
    G_nR_n[2] = G_n[2]*R_n[0];
    G_nR_n[3] = 0.0;
    G_nR_n[4] = 0.0;
    G_nR_n[5] = G_n[5]*R_n[3]; //3x2

    transposeMatrix3x3(G_n_T, G_n); //2x3
//    R[0] = G_nR_n[0] * G_n_T[0];
//    R[1] = G_nR_n[0] * G_n_T[1];
//    R[2] = 0.0;
//    R[3] = G_nR_n[2] * G_n_T[0];
//    R[4] = G_nR_n[2] * G_n_T[1];
//    R[5] = 0.0;
//    R[6] = 0.0;
//    R[7] = 0.0;
//    R[8] = G_nR_n[5] * G_n_T[5]; //3x3

    matmul3x3(GP_, G, P_);
    transposeMatrix3x3(GT, G);
    matmul3x3(G_P_Gt, GP_, GT);
    addMatrix3x3(P_, G_P_Gt, R);
}

//3 diaforetika measurement models->3 diaforetika update lidar
void ExtendedKalmanFilter::update_lidar(const double l_actual[2], const double xlidar, const double ylidar) {
    //theoroume yaw oti einai swsto-->correct east, north
    double theta2 = x_[2];
    double l_pred[2] = {x_[0], x_[1]}; //predicted
//    cout<<"theta2: "<<theta2<<endl;
    double H_l[6] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0}; //2x3
//    double H_l[6] = {1.0, 0.0, -(ylidar * std::sin(theta2)) + (xlidar * std::cos(theta2)),
//                     0.0, 1.0, + (ylidar * std::cos(theta2)) + (xlidar * std::sin(theta2))}; //2x3
//    double H_l[6] = {1.0, 0.0, +(ylidar * std::sin(theta2)) - (xlidar * std::cos(theta2)),
//                     0.0, 1.0, -(ylidar * std::cos(theta2)) - (xlidar * std::sin(theta2))}; //2x3
    double HT_l[6], HP_l[6], HPHt_l[4], HPHt_Q_inv_l[4], Kl[6], K_mult_z_diff_l[3], KH_l[9], KHP_l[9],HPHt_Q_l[2],P_Ht_l[6], Hx[2], Il_KH_l[9];
    double Il[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

//    cout<<"ekf yaw: "<<normalize_angles(x_[2])<<endl;
    transposeMatrix3x2(HT_l, H_l);
    Mat2x3_Mul_Mat3(HP_l,H_l,P_);//H 2X3 P_ 3X3 -->2X3
    matmul2x3x3(HPHt_l,HP_l,HT_l); //HP_ 2X3 HT 3x2-->2x2 //lathos
    addMatrix2x2(HPHt_Q_l, HPHt_l, Ql);
    inverseMatrix2x2(HPHt_Q_inv_l, HPHt_Q_l);
    matmul3x3x2(P_Ht_l, P_, HT_l); //P_ 3X3, HT 3x2--->3x2
    matmul3x2x2(Kl, P_Ht_l, HPHt_Q_inv_l); //P_Ht 3X2, HPHt_Q_inv 2x2-->3X2
    double z_diff_l[2] = {l_actual[0] - l_pred[0], l_actual[1] - l_pred[1]}; //actual - predicted
    matvecmul3x2(K_mult_z_diff_l, Kl, z_diff_l); //K 3X2 z_diff 2X1 -->3X1
    addVectors3D(x_, x_, K_mult_z_diff_l);
    x_[2] = normalize_angles(x_[2]);
    // Now only update x and y in the state vector
//    x_[0] += K_mult_z_diff_l[0];
//    x_[1] += K_mult_z_diff_l[1];
//    x_[2] = x_[2];

    Mat3_AxB(KH_l, Kl, H_l); //K 3X2 H 2X3-->3X3
    subtractMatrix3x3(Il_KH_l, Il, KH_l);
    matmul3x3(P_, Il_KH_l, P_);
//    matmul3x3(KHP_l,KH_l,P_);  //KH 3X3 P_ 3X3 --> 3X3
//    subtractMatrix3x3(P_, P_, KHP_l);
}

//void ExtendedKalmanFilter::update_lidar(const double l_gt[2], const double l_obs[2], const double xlidar, const double ylidar) {
//    double theta2 = x_[2];
//    double rx = xlidar * std::sin(theta2) + ylidar * std::cos(theta2);
//    double ry = -xlidar * std::cos(theta2) + ylidar * std::sin(theta2);
//    double q_actual = pow(l_gt[0] - rx, 2) + pow(l_gt[1] - ry, 2);
//    double q_pred = pow(l_obs[0] - rx, 2) + pow(l_obs[1] - ry, 2);
//    double dx = l_obs[0] - rx;
//    double dy = l_obs[1] - ry;
//    double z[2] = {sqrt(q_actual) , atan2(l_gt[1] - ry,l_gt[0] - rx) - theta2};
//    double z_[2] = {sqrt(q_pred) , atan2(l_obs[1] - ry,l_obs[0] - rx) - theta2};
//    double H_l[6] = {0, 0, ( (dx * (-xlidar * cos(theta2) + ylidar * sin(theta2))) + (dy * (-xlidar * sin(theta2) - ylidar * cos(theta2))) / sqrt(q_pred) ),
//                     0, 0, ( (dy * (-xlidar * cos(theta2) + ylidar * sin(theta2))) - (dx * (-xlidar * sin(theta2) - ylidar * cos(theta2))) / (pow(dx,2) + pow(dy, 2)) ) - 1}; //2x3
//
//    double HT_l[6], HP_l[6], HPHt_l[4], HPHt_Q_inv_l[4], Kl[6], K_mult_z_diff_l[3], KH_l[9], KHP_l[9],HPHt_Q_l[2],P_Ht_l[6],Il_KH_l[9];
//    double Il[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
//    transposeMatrix3x2(HT_l, H_l);
//    Mat2x3_Mul_Mat3(HP_l,H_l,P_);//H 2X3 P_ 3X3 -->2X3
//    matmul2x3x3(HPHt_l,HP_l,HT_l); //HP_ 2X3 HT 3x2-->2x2 //lathos
//    addMatrix2x2(HPHt_Q_l, HPHt_l, Ql);
//    inverseMatrix2x2(HPHt_Q_inv_l, HPHt_Q_l);
//    matmul3x3x2(P_Ht_l, P_, HT_l); //P_ 3X3, HT 3x2--->3x2
//    matmul3x2x2(Kl, P_Ht_l, HPHt_Q_inv_l); //P_Ht 3X2, HPHt_Q_inv 2x2-->3X2
//    double z_diff_l[2] = {z[0] - z_[0], z[1] - z_[1]}; //actual - predicted
//    matvecmul3x2(K_mult_z_diff_l, Kl, z_diff_l); //K 3X2 z_diff 2X1 -->3X1
//    addVectors3D(x_, x_, K_mult_z_diff_l);
//    // Now only update x and y in the state vector
////    x_[0] += K_mult_z_diff_l[0];
////    x_[1] += K_mult_z_diff_l[1];
////    x_[2] = x_[2];
////    cout<<"theta2 after: "<<x_[2]<<endl;
//    Mat3_AxB(KH_l, Kl, H_l); //K 3X2 H 2X3-->3X3
////    matmul3x3(KHP_l,KH_l,P_);  //KH 3X3 P_ 3X3 --> 3X3
////    subtractMatrix3x3(P_, P_, KHP_l);
//    subtractMatrix3x3(Il_KH_l, Il, KH_l);
//    matmul3x3(P_, Il_KH_l, P_);
//}

//void ExtendedKalmanFilter::update_lidar(const double l_gt[2], const double l_obs[2], const double xlidar, const double ylidar) {
//    double theta2 = normalize_angles(x_[2]);
//    double rx = xlidar * std::sin(theta2) + ylidar * std::cos(theta2);
//    double ry = -xlidar * std::cos(theta2) + ylidar * std::sin(theta2);
//    double q_actual = pow(rx, 2) + pow(ry, 2);
//    double q_pred = pow(l_gt[0] - x_[0], 2) + pow(l_gt[1] - x_[1], 2);
//    double dx = l_gt[0] - x_[0];
//    double dy = l_gt[1] - x_[1];
//    double z[2] = {sqrt(q_actual) , atan2(ry,rx) - theta2};
//    double z_[2] = {sqrt(q_pred) , atan2(l_gt[1] - x_[1],l_gt[0] - x_[0]) - theta2};
//    double H_l[6] = {-dx/sqrt(q_pred), -dy/sqrt(q_pred), 0.0, -dy/q_pred, dx/q_pred, -1}; //2x3
//
//    double HT_l[6], HP_l[6], HPHt_l[4], HPHt_Q_inv_l[4], Kl[6], K_mult_z_diff_l[3], KH_l[9], KHP_l[9],HPHt_Q_l[2],P_Ht_l[6],Il_KH_l[9];
//    double Il[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
//    transposeMatrix3x2(HT_l, H_l);
//    Mat2x3_Mul_Mat3(HP_l,H_l,P_);//H 2X3 P_ 3X3 -->2X3
//    matmul2x3x3(HPHt_l,HP_l,HT_l); //HP_ 2X3 HT 3x2-->2x2 //lathos
//    addMatrix2x2(HPHt_Q_l, HPHt_l, Ql);
//    inverseMatrix2x2(HPHt_Q_inv_l, HPHt_Q_l);
//    matmul3x3x2(P_Ht_l, P_, HT_l); //P_ 3X3, HT 3x2--->3x2
//    matmul3x2x2(Kl, P_Ht_l, HPHt_Q_inv_l); //P_Ht 3X2, HPHt_Q_inv 2x2-->3X2
//    double z_diff_l[2] = {z[0] - z_[0], z[1] - z_[1]}; //actual - predicted
//    matvecmul3x2(K_mult_z_diff_l, Kl, z_diff_l); //K 3X2 z_diff 2X1 -->3X1
//    addVectors3D(x_, x_, K_mult_z_diff_l);
//    // Now only update x and y in the state vector
////    x_[0] += K_mult_z_diff_l[0];
////    x_[1] += K_mult_z_diff_l[1];
////    x_[2] = x_[2];
////    cout<<"theta2 after: "<<x_[2]<<endl;
//    Mat3_AxB(KH_l, Kl, H_l); //K 3X2 H 2X3-->3X3
////    matmul3x3(KHP_l,KH_l,P_);  //KH 3X3 P_ 3X3 --> 3X3
////    subtractMatrix3x3(P_, P_, KHP_l);
//    subtractMatrix3x3(Il_KH_l, Il, KH_l);
//    matmul3x3(P_, Il_KH_l, P_);
//}


