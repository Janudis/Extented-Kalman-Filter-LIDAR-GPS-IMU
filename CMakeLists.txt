cmake_minimum_required(VERSION 3.25)
project(fusion_camera_gps_imu)

set(CMAKE_CXX_STANDARD 17)

add_executable(fusion_camera_gps_imu main.cpp
        ekf4.cpp
        ekf.h
        geo_ned.cpp
        geo_ned.h
        utm.h utm.cpp matrix_operations.h matrix_operations.cpp data_preprocessing.cpp data_preprocessing.h)
