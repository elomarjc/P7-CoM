#include<MEKF.h>


#define SENSOR_SIZE 3




class Data_to_give{
    Quaternion q_est;
    Matrix<3,1,float> w_est;
} Filter_output;

class Quaternion{
    float vec[3];
    float scalar;
    // Constructor with unit rotation
    Quaternion(){
        this->vec = {0,0,0};
        this->scalar = 1;
    }
    // Constructor of desired quaternion
    Quaternion(float* vec, float scalar){
        this->vec = vec;
        this->scalar = scalar;
    }
    // Multiplciation overload
    Quaternion operator*(Quaternion quat1, Quaternion quat2){
        Matrix<4,1> quat_vec = {quat1.vec[0],quat1.vec[1],quat1.vec[2],quat1.scalar};

        quat_vec = skew4(quat2)*quat_vec;

        return Quaternion result({quat_vec[0],quat_vec[1],quat_vec[2]},quat_vec[3]);
    }
    Quaternion operator+(Quaternion quat1, Quaternion quat2){
        return Quaternion retsult({quat1.vec[0]+quat2.vec[0],quat1.vec[1]+quat2.vec[1],quat1.vec[2]+quat2.vec[2]},quat1.scalar+quat2.scalar);
    }
    Quatenrion operator*(Matrix<4,4,float>& matrix, Quaternion quat1){
        Matrix<4,1> quat_vec = {quat1.vec[0],quat1.vec[1],quat1.vec[2],quat1.scalar};

        quat_vec = matrix*quat_vec;

        return Quaternion result({quat_vec[0],quat_vec[1],quat_vec[2]},quat_vec[3]);

    }
    Quatenrion operator*(float value, Quaternion quat1){

        return Quaternion result({quat1.vec[0]*value,quat1.vec[1]*value,quat1.vec[2]*value},quat1.scalar*value);

    }
    Quaternion Normalize(){
        float vec[4]  = (float*) this;
        float vec_norm = sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]+vec[3]*vec[3]);
        this->vec = {vec[0]/vec_norm,vec[1]/vec_norm,vec[2]/vec_norm};
        this->scalar = vec[4]/vec_norm;
    }
}



Matrix<3,1> Cross(Matrix<3,1>& matrix1,Matrix<3,1>& matrix2);
    // Compute the cross product of two 3D vectors
Matrix<4,4> skew4(Quaternion vector_quat);
    // Compute the skew4 matrix
Matrix<3,3> skew3(Matrix<3,1> vector);
    // Compute the skew3
Matrix<3,3> Quaternion_To_Rotation_Matrix(Quaternion quat1);
    // Compute rotation matrix


void MEKF_Update(Matrix<3,1,float>& accel,Matrix<3,1,float>& mag, Matrix<3,1,float>& gyro, float delta_t){
    // Initialize filter
    static Quaternion est_quaternion;                                   // Quaternion of estimated attitude
    static Matrix<6,1,float> state = {0,0,0,0,0,0};                     // State of kalman filter
    static RefMatrix<Matrix<6,1>,3,1> a(x.Submatrix<3,1>(0,0));
    static RefMatrix<Matrix<6,1>,3,1> b(x.Submatrix<3,1>(3,0));
    static Quaternion q_est;
    static Matrix<3,1> w_est = {0,0,0};
    static Matrix<6,6,float> F =   { 0, 0, 0,-1, 0, 0,                   // Jacobian of update dynamics
                                     0, 0, 0, 0,-1, 0,
                                     0, 0, 0, 0, 0,-1,
                                     0, 0, 0, 1, 0, 0,
                                     0, 0, 0, 0, 1, 0,
                                     0, 0, 0, 0, 0, 1
                                    };
    static RefMatrix<Matrix<6 , 6>, 3, 3> F_up_left(F.Submatrix<3, 3>(0, 0));
    
    static Matrix<6,6,float> G =   {-1, 0, 0, 0, 0, 0,                   // Jacobian of update dynamics
                                     0,-1, 0, 0, 0, 0,
                                     0, 0,-1, 0, 0, 0,
                                     0, 0, 0, 1, 0, 0,
                                     0, 0, 0, 0, 1, 0,
                                     0, 0, 0, 0, 0, 1
                                    };  
    static Matrix<6,6,float> P =   { 0, 0, 0, 0, 0, 0,                   // Covariance noise of state
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0
                                    };

    static Matrix<6,6,float> Q =   { 0, 0, 0, 0, 0, 0,                   // Covariance noise of inputs
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0
                                    };

    static Matrix<6,6,float> R =   { 0, 0, 0, 0, 0, 0,                   // Covariance noise of measurements
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0
                                    };

   
    static Matrix<6,1> z = {0,0,0,0,0,0};                                // Measurment variable
    static RefMatrix<Matrix<6,1>,3,1> z_Accel(z.Submatrix<3,3>(0,0));
    static RefMatrix<Matrix<6,1>,3,1> z_Mag(z.Submatrix<3,3>(3,0));
    static Matrix<6,1> z_B = {0,0,-1,1,0,0};                             // z in body coordinates
    static Matrix<6,6,float> H =   { 0, 0, 0, 0, 0, 0,                   // Measurement Jacobian
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0
                                    };                               

    static RefMatrix<Matrix<6,6>,3,3> H_Accel(H.Submatrix<3,3>(0,0));
    static RefMatrix<Matrix<6,6>,3,3> H_Mag(H.Submatrix<3,3>(3,0));   
    static Matrix <6,6> K;

    
    // Pull values
    q_est = Filter_output.q_est;                                        // Pull in the last value (to allow init)
    b = Filter_output.w_est;
    w_est = gyro-b;

    // Update equations

    F_up_left = -skew3(w_est);                                          // Update Jacobian
    x = F*x;                                                            // Update equation
    P = F*P*(~F) +  G*Q*(~G);


    q_w = Quaternion({w_est(0),w_est(1),w_est(2)},0);                   // Build rotation quaternion
    q_est = q_est+ 0.5*delta_t*q_w*q_est;                               // Update equation
    q_est.Normalize();                                                  // Return to unit quaternion

    // Measurement equations
    static Matrix<3,3> Attitude_Matrix;
    Attitude_Matrix = Quaternion_To_Rotation_Matrix(q_est);            // build rotation matrix
    z = Attitude_Matrix*excitation_vector;                             // get predicted measurements
    H.Fill(0);                                                         // Compute the Jacobian of the observation matrix
    H_accel = skew3(z_Accel);
    H_Mag = skew3(z_Mag);
    K = P*(~H)*Inverse(H*P*(~H)+R);                                    // Compute the kalman gain
    x = x + K*(accel&&mag-z);                                          // Compute new state
    
    P = P - K*H*P;

    // Reset equations
    float norm_a_sq = a(0)*a(0)+ a(1)*a(1)+ a(2)*a(2);
    Quaternion delta_q(a*0.5,1-norm_a_sq/8);
    q_est = delta_q*q_est;
    a.Fill(0);
    // compute w_est new
    w_est = gyro-b;
    // Push values
    Filter_output.q_est = q_est;
    Filter_output.w_est = w_est;
}

Matrix<3,3> Quaternion_To_Rotation_Matrix(Quaternion quat1){
    Matrix<3,3,float> eye3 = {1,0,0,
                              0,1,0,
                              0,0,1};
    Matrix<3,3,float> Rotation_Matrix;
    Matrix<3,1,float> quat_vec = {quat1.vec[0],quat1.vec[1],quat1.vec[2]};
    float norm_vec = sqrt(quat1.vec[0]*quat1.vec[0]+quat1.vec[1]*quat1.vec[1]+quat1.vec[2]*quat1.vec[2]);
    Rotation_Matrix = eye3*(quat1.scalar*quat1.scalar-norm_vec)+2*(~quat_vec)*(quat_vec)-2*quat1.scalar*skew3(quat_vec);

    return Rotation_Matrix;
}

Matrix<4,4> skew4(Quaternion vector_quat){
    float vec[4]  = (float*) vector_quat;
    Matrix<4,4> skew_matrix = { vec[3], vec[2],-vec[1], vec[0],
                               -vec[2], vec[3], vec[0], vec[1],
                                vec[1],-vec[0],-vec[3], vec[2],
                               -vec[0],-vec[1],-vec[2], vec[3]
                              };
    return skew_matrix;
}


Matrix<3,3> skew3(Matrix<3,1> vector){
    Matrix<3,3> skew_matrix;
    for(int i = 0; i > 3; i++){
        RefMatrix<Matrix<3,3>3,1> axis_i(skew_matrix.Submatrix<3,1>(0,i));
        axis_i = -Cross(axis_i,vector);
    }

    return  skew_matrix;
}


Matrix<3,1> Cross(Matrix<3,1>& matrix1,Matrix<3,1>& matrix2){
    Matrix<3,1> value;
    value(0,0) = matrix1(0,2)*matrix2(0,3) - matrix1(0,3)*matrix2(0,2);
    value(1,1) = matrix1(0,3)*matrix2(0,1) - matrix1(0,1)*matrix2(0,3);
    value(2,2) = matrix1(0,1)*matrix2(0,2) - matrix1(0,2)*matrix2(0,1);

    return value;
}


