#include "matrix_operations.h"
// matmul3x3: multiplies two 3x3 matrices.
void matmul3x3(double res[9], const double A[9], const double B[9])
{
    res[0] = A[0] * B[0] + A[1] * B[3] +A[2] * B[6];
    res[1] = A[0] * B[1] + A[1] * B[4] +A[2] * B[7];
    res[2] = A[0] * B[2] + A[1] * B[5] +A[2] * B[8];
    res[3] = A[3] * B[0] + A[4] * B[3] +A[5] * B[6];
    res[4] = A[3] * B[1] + A[4] * B[4] +A[5] * B[7];
    res[5] = A[3] * B[2] + A[4] * B[5] +A[5] * B[8];
    res[6] = A[6] * B[0] + A[7] * B[3] +A[8] * B[6];
    res[7] = A[6] * B[1] + A[7] * B[4] +A[8] * B[7];
    res[8] = A[6] * B[2] + A[7] * B[5] +A[8] * B[8];
}

// matvecmul3x3: multiplies a 3x3 matrix with a 3x1 vector.
void matvecmul3x3(double res[3], const double A[9], const double B[3])
{
    res[0] = A[0 + 0 * 3] * B[0] + A[0 + 1 * 3] * B[1] + A[0 + 2 * 3] * B[2];
    res[1] = A[1 + 0 * 3] * B[0] + A[1 + 1 * 3] * B[1] + A[1 + 2 * 3] * B[2];
    res[2] = A[2 + 0 * 3] * B[0] + A[2 + 1 * 3] * B[1] + A[2 + 2 * 3] * B[2];
}

//3x3 x 3x2
void matmul3x3x2(double res[6], const double A[9], const double B[6]) {
    res[0] = A[0]*B[0] + A[1]*B[2] +A[2]*B[4];
    res[1] = A[0]*B[1] + A[1]*B[3] +A[2]*B[5];
    res[2] = A[3]*B[0] + A[4]*B[2] +A[5]*B[4];
    res[3] = A[3]*B[1] + A[4]*B[3] +A[5]*B[5];
    res[4] = A[6]*B[0] + A[7]*B[2] +A[8]*B[4];
    res[5] = A[6]*B[1] + A[7]*B[3] +A[8]*B[5];
}

//3x2 x 2x3
void Mat3_AxB(double res[9], const double A[6], const double B[6])
{
    res[0] = A[0]*B[0] + A[1]*B[3];
    res[1] = A[0]*B[1] + A[1]*B[4];
    res[2] = A[0]*B[2] + A[1]*B[5];
    res[3] = A[2]*B[0] + A[3]*B[3];
    res[4] = A[2]*B[1] + A[3]*B[4];
    res[5] = A[2]*B[2] + A[3]*B[5];
    res[6] = A[4]*B[0] + A[5]*B[3];
    res[7] = A[4]*B[1] + A[5]*B[4];
    res[8] = A[4]*B[2] + A[5]*B[5];
}

void transposeMatrix3x2(double res[6], const double A[6])
{
    res[0] = A[0];
    res[1] = A[3];
    res[2] = A[1];
    res[3] = A[4];
    res[4] = A[2];
    res[5] = A[5];
}

void transposeMatrix3x3(double res[9], const double A[9])
{
    res[0 + 0 * 3] = A[0 + 0 * 3];
    res[1 + 0 * 3] = A[0 + 1 * 3];
    res[2 + 0 * 3] = A[0 + 2 * 3];

    res[0 + 1 * 3] = A[1 + 0 * 3];
    res[1 + 1 * 3] = A[1 + 1 * 3];
    res[2 + 1 * 3] = A[1 + 2 * 3];

    res[0 + 2 * 3] = A[2 + 0 * 3];
    res[1 + 2 * 3] = A[2 + 1 * 3];
    res[2 + 2 * 3] = A[2 + 2 * 3];
}

//2x3 x 3x2
void matmul2x3x3(double res[4], const double A[6], const double B[6])
{
    res[0]= A[0]*B[0] + A[1]*B[2] + A[2]*B[4];
    res[1] = A[0]*B[1] + A[1]*B[3] + A[2]*B[5];
    res[2] = A[3]*B[0] + A[4]*B[2] +A[5]*B[4];
    res[3] = A[3]*B[1] + A[4]*B[3] + A[5]*B[5];
}

void inverseMatrix2x2(double res[4], const double A[4]) {
    double det = A[0 + 0 * 2] * A[1 + 1 * 2] - A[0 + 1 * 2] * A[1 + 0 * 2];
    double inv_det = 1.0 / det;

    res[0 + 0 * 2] =  A[1 + 1 * 2] * inv_det;
    res[0 + 1 * 2] = -A[0 + 1 * 2] * inv_det;
    res[1 + 0 * 2] = -A[1 + 0 * 2] * inv_det;
    res[1 + 1 * 2] =  A[0 + 0 * 2] * inv_det;
}

void addMatrix2x2(double res[4], const double A[4], const double B[4]) {
    res[0 + 0 * 2] = A[0 + 0 * 2] + B[0 + 0 * 2];
    res[0 + 1 * 2] = A[0 + 1 * 2] + B[0 + 1 * 2];
    res[1 + 0 * 2] = A[1 + 0 * 2] + B[1 + 0 * 2];
    res[1 + 1 * 2] = A[1 + 1 * 2] + B[1 + 1 * 2];
}

//3x2 x 2x2
void matmul3x2x2(double res[6], const double A[6], const double B[4]) {
    res[0] = A[0]*B[0] + A[1]*B[2];
    res[1] = A[0]*B[1] + A[1]*B[3];
    res[2] = A[2]*B[0] + A[3]*B[2];
    res[3] = A[2]*B[1] + A[3]*B[3];
    res[4] = A[4]*B[0] + A[5]*B[2];
    res[5] = A[4]*B[1] + A[5]*B[3];

}

// 3x2 x 2x1
void matvecmul3x2(double res[3], const double A[6], const double B[2]) {
    res[0] = A[0]*B[0] + A[1]*B[1];
    res[1] = A[2]*B[0] + A[3]*B[1];
    res[2] = A[4]*B[0] + A[5]*B[1];
}

void addVectors3D(double res[3], const double A[3], const double B[3]) {
    res[0] = A[0] + B[0];
    res[1] = A[1] + B[1];
    res[2] = A[2] + B[2];
}

void subtractMatrix3x3(double res[9], const double A[9], const double B[9]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            res[i + j * 3] = A[i + j * 3] - B[i + j * 3];
        }
    }
}

void addMatrix3x3(double res[9], const double A[9], const double B[9]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            res[i + j * 3] = A[i + j * 3] + B[i + j * 3];
        }
    }
}

//2x3 x 3x3
void Mat2x3_Mul_Mat3(double res[6], const double A[6], const double B[9])
{
    res[0] = A[0]*B[0] +A[1]*B[3] + A[2]*B[6];
    res[1] = A[0]*B[1] +A[1]*B[4] +A[2]*B[7];
    res[2] = A[0]*B[2] +A[1]*B[5] +A[2]*B[8];
    res[3] = A[3]*B[0] +A[4]*B[3] +A[5]*B[6];
    res[4] = A[3]*B[1] +A[4]*B[4] +A[5]*B[7];
    res[5] = A[3]*B[2] +A[4]*B[5] +A[5]*B[8];

}



