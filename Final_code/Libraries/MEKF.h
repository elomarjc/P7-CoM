#ifndef MEKF_H
#define MEKF_H

#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <Math.h>



class Quaternion{
  public:
    Matrix<3,1> vec;
    float scalar;
    // Constructor with unit rotation
    Quaternion();
    // Constructor of desired quaternion
    Quaternion(float* vec_in, float scalar_in);
    Quaternion(Matrix<3,1,float>& vec_in, float scalar_in);
    Quaternion(Matrix<4,1,float>& vec_in);
    // Normalizes quaternion
    void Normalize();

    
};
Quaternion operator*(Quaternion quat1, Quaternion quat2);
Quaternion operator+(Quaternion quat1, Quaternion quat2);
Quaternion operator*(float value, Quaternion quat1);
Quaternion operator*(Quaternion quat1,float value);


Matrix<3,3> Quaternion_To_Rotation_Matrix(Quaternion quat1);
    // Compute rotation matrix
Matrix<3,1,float> Cross(Matrix<3,1,float>& matrix1,Matrix<3,1,float>& matrix2);
    // Compute the cross product of two 3D vectors
Matrix<4,4> skew4(Quaternion quat);
    // Compute the skew4 matrix
Matrix<3,3> skew3(Matrix<3,1,float> vector);

Matrix<3,1,float> MEKF_GET_rot_speed();

Quaternion MEKF_GET_quaternion();

void MEKF_Update(Matrix<3,1,float>& accel,Matrix<3,1,float>& gyro, Matrix<3,1,float>& mag);

void MEKF_Innit_dt(float dt);

void MEKF_Innit_q(Quaternion q);

void MEKF_Innit_bias(Matrix<6,1,float>& b);




#endif