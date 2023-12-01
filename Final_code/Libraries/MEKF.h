#ifndef MEKF_H
#define MEKF_H

#include <BasicLinearAlgebra.h>
using namespace BLA;
#include<Math.h>

void MEKF_Update(Matrix<6,1,float>& accel,Matrix<6,1,float>& mag, Matrix<6,1,float>& gyro);
    // COmputes the enxt step of the MEKF.

Matrix<3,3> Quaternion_To_Rotation_Matrix(Quaternion quat1);
    // Compute rotation matrix

Matrix<3,1> Cross(Matrix<3,1>& matrix1,Matrix<3,1>& matrix2);
    // Compute the cross product of two 3D vectors
Matrix<4,4> skew4(Quaternion vector_quat);
    // Compute the skew4 matrix
Matrix<3,3> skew3(Matrix<3,1> vector);
    // Compute the skew3


#endif