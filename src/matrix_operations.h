//
// Created by user on 28/5/2023.
//

#ifndef MATRIX_OPERATIONS_HPP
#define MATRIX_OPERATIONS_HPP

void matmul3x3(double res[9], const double A[9], const double B[9]);
void matvecmul3x3(double res[3], const double A[9], const double B[3]);
void matmul3x3x2(double res[6], const double A[9], const double B[6]);
void Mat3_AxB(double res[9], const double A[6], const double B[6]);
void transposeMatrix3x2(double res[6], const double A[6]);
void transposeMatrix3x3(double res[9], const double A[9]);
void matmul2x3x3(double res[4], const double A[6], const double B[6]);
void matmul3x2x2(double res[6], const double A[6], const double B[4]);
void matvecmul3x2(double res[3], const double A[6], const double B[2]);
void Mat2x3_Mul_Mat3(double res[6], const double A[6], const double B[9]);
void inverseMatrix2x2(double res[4], const double A[4]);
void addMatrix2x2(double res[4], const double A[4], const double B[4]);
void addVectors3D(double res[3], const double A[3], const double B[3]);
void subtractMatrix3x3(double res[9], const double A[9], const double B[9]);
void addMatrix3x3(double res[9], const double A[9], const double B[9]);

#endif //MATRIX_OPERATIONS_HPP
