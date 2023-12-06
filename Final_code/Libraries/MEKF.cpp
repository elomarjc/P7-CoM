#include"MEKF.h"


Quaternion::Quaternion(){
  this->vec.Fill(0);
  this->scalar = 1;
}

Quaternion::Quaternion(float* vec_in, float scalar_in){
  Matrix<3,1> aux = {vec_in[0],vec_in[1],vec_in[2]};
  this->vec = aux;
  this->scalar = scalar_in;
}

Quaternion::Quaternion(Matrix<3,1,float>& vec_in, float scalar_in){
  this->vec = vec_in;
  this->scalar = scalar_in;

}

Quaternion::Quaternion(Matrix<4,1,float>& vec_in){
  RefMatrix<Matrix<4,1>,3,1> a(vec_in.Submatrix<3,1>(0,0));
  this->vec = a;
  this->scalar = vec_in(3);
}
void Quaternion::Normalize(){
  Matrix<1,1,float> vec_norm_sq = (~this->vec)*this->vec;
  float vec_norm = sqrt(vec_norm_sq(0)+this->scalar*this->scalar);
  this->vec = this->vec*(1/vec_norm);
  this->scalar = this->scalar/vec_norm;
}

Quaternion operator*(Quaternion quat1, Quaternion quat2){
  Matrix<1,1,float> scalar = {quat1.scalar};
  Matrix<4,1,float> quat_vec = quat1.vec&&scalar;
  quat_vec = skew4(quat2)*quat_vec;

  Quaternion result(quat_vec);

  return result;
}

Quaternion operator+(Quaternion quat1, Quaternion quat2){
  Matrix<3,1,float>sum = quat1.vec+quat2.vec;
  Quaternion result(sum,quat1.scalar+quat2.scalar);
  return result;
}


Quaternion operator*(float value, Quaternion quat1){
    float product[3];
  for(int i = 1 ; i<3 ; i++){
    product[i] = quat1.vec(i)*value;
  }
  Quaternion result(product,quat1.scalar*value);
  return result;

}
Quaternion operator*(Quaternion quat1,float value){
    float product[3];
  for(int i = 1 ; i<3 ; i++){
    product[i] = quat1.vec(i)*value;
  }
  Quaternion result(product,quat1.scalar*value);
  return result;

}


class Data_to_give{
    public:
    Quaternion q_est;
    Matrix<3,1,float> w_est;
    float dt;
    Matrix<6,1,float> x = {0,0,0,0,0,0};
} Filter_settings;





Quaternion MEKF_GET_quaternion(){
    return Filter_settings.q_est;
}

Matrix<3,1,float> MEKF_GET_rot_speed(){
  return Filter_settings.w_est;
}

void MEKF_Innit_dt(float dt){
  Filter_settings.dt = dt;
}

void MEKF_Innit_q(Quaternion q){
  Filter_settings.q_est = q;
}

void MEKF_Innit_bias(Matrix<6,1,float>& b){
  Filter_settings.x(3) = b(3);
  Filter_settings.x(4) = b(4);
  Filter_settings.x(5) = b(5);
}

void MEKF_Update(Matrix<3,1,float>& accel,Matrix<3,1,float>& gyro, Matrix<3,1,float>& mag,float dt){
    // Initialize filter
  static Matrix<6,1,float> x = Filter_settings.x;                         // x of kalman filter
  static RefMatrix<Matrix<6,1>,3,1> a(x.Submatrix<3,1>(0,0));
  static RefMatrix<Matrix<6,1>,3,1> b(x.Submatrix<3,1>(3,0));
  static Quaternion q_est = Filter_settings.q_est;
  static Matrix<3,1,float> w_est = {0,0,0};
  static Matrix<6,6,float> F =   { 0, 0, 0,-1, 0, 0,                   // Jacobian of update dynamics
                                    0, 0, 0, 0,-1, 0,
                                    0, 0, 0, 0, 0,-1,
                                    0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0,
                                    0, 0, 0, 0, 0, 1
                                  };
  static RefMatrix<Matrix<6 , 6,float>, 3, 3> F_up_left(F.Submatrix<3, 3>(0, 0));
  
  static Matrix<6,6,float> G =   {-1, 0, 0, 0, 0, 0,                   // Jacobian of update dynamics
                                    0,-1, 0, 0, 0, 0,
                                    0, 0,-1, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0,
                                    0, 0, 0, 0, 0, 1
                                  };  
  static Matrix<6,6,float> P =   { 2, 0, 0, 0, 0, 0,                   // Covariance noise of state
                                    0, 2, 0, 0, 0, 0,
                                    0, 0, 2, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0
                                  };

  static Matrix<6,6,float> Q =   {0.2601,    0,    0,    0,    0,    0,                   // Covariance noise of inputs
                                      0,0.2827,    0,    0,    0,    0,
                                      0,    0,0.2217,    0,    0,    0,
                                      0,    0,    0,0.1745,    0,    0,
                                      0,    0,    0,    0,0.1745,    0,
                                      0,    0,    0,    0,    0,0.1745
                                  };

  static Matrix<6,6,float> R =   {  10,    0,    0,    0,    0,    0,                   // Covariance noise of measurements
                                      0,  10,    0,    0,    0,    0,
                                      0,    0,  10,    0,    0,    0,
                                      0,    0,    0,  10,    0,    0,
                                      0,    0,    0,    0,  10,    0,
                                      0,    0,    0,    0,    0,  10
                                  };

  
  static Matrix<6,1,float> z = {0,0,0,0,0,0};                          // Measurment variable
  static RefMatrix<Matrix<6,1>,3,1> z_Accel(z.Submatrix<3,1>(0,0));
  static RefMatrix<Matrix<6,1>,3,1> z_Mag(z.Submatrix<3,1>(3,0));
  static Matrix<3,1,float> z_Accel_B = {0,0,-1};
  static Matrix<3,1,float> z_Mag_B = {0.3245,0,-0.9459};              // Use is as x axis, but remember that it is rotated downwards
                        // z in body coordinates
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
  w_est = gyro-b;

  // Update equations
  
  //F_up_left = -skew3(w_est);                                          // Update Jacobian
  //x = F*x;                                                            // Update equation
  static Matrix<6,6> eye_dt = {dt,0,0,0,0,0,0,dt,0,0,0,0,0,0,dt,0,0,0,0,0,0,dt,0,0,0,0,0,0,dt,0,0,0,0,0,0,dt};
  P = P + eye_dt*(F*P+ P*(~F) +  G*Q*(~G));                                           // Update the autocovariance matrix
  
  Quaternion q_w(w_est,0);                                            // Build rotation quaternion
  Matrix<4,1,float> q_vec =  {q_est.vec(0),q_est.vec(1),q_est.vec(2),q_est.scalar};
  Matrix<4,4,float> q_w_skew;
  Matrix<4,4,float> aux = {1/2*dt,0,0,0,0,1/2*dt,0,0,0,0,1/2*dt,0,0,0,0,1/2*dt};
  q_w_skew = skew4(q_w);
  q_vec = q_vec+ aux*q_w_skew*q_vec;                                    // Update equation
  q_est = Quaternion(q_vec);
  q_est.Normalize();                                                  // Return to unit quaternion

  // Measurement equations
  
  static Matrix<3,3> Attitude_Matrix;
  Attitude_Matrix = Quaternion_To_Rotation_Matrix(q_est);            // build rotation matrix
  z_Accel = Attitude_Matrix*z_Accel_B;                             // get predicted measurements
  z_Mag = Attitude_Matrix*z_Mag_B;
  //Serial << accel << "," << mag << gyro <<"\n";
  //Serial << z_Accel << "," << z_Mag << w_est << "\n";
  
  H.Fill(0);                                                         // Compute the Jacobian of the observation matrix
  H_Accel = skew3(z_Accel);
  H_Mag = skew3(z_Mag);
  K = P*(~H)*Inverse(H*P*(~H)+R);                                    // Compute the kalman gain
  x = x + K*((accel&&mag)-z);                                          // Compute new state
  
  P = P - K*H*P;
  // Reset equations
  float norm_a_sq = a(0,0)*a(0,0)+ a(1,0)*a(1,0)+ a(2,0)*a(2,0);
  //Serial << b << sqrt(norm_a_sq) <<"\n";
  float a_vec[] = {a(0,0)/2,a(1,0)/2,a(2,0)/2};
  Quaternion delta_q(a_vec,(float) (1-norm_a_sq/8));
  q_est = delta_q*q_est;
  q_est.Normalize(); 
  a(0,0) = 0;
  a(1,0) = 0;
  a(2,0) = 0;
  
  // compute w_est new
  w_est = gyro-b;
  // Push values
  Filter_settings.q_est = q_est;
  Filter_settings.w_est = w_est;
  
}



Matrix<3,3> operator*(Matrix<3,3> matrix, float scalar){
    Matrix <3,3> eye = {scalar,0,0,0,scalar,0,0,0,scalar};
    return eye*matrix;
}

Matrix<3,3> operator*(float scalar,Matrix<3,3> matrix){
    Matrix <3,3> eye = {scalar,0,0,0,scalar,0,0,0,scalar};
    return eye*matrix;
}
Matrix<3,3> operator*(Matrix<3,3> matrix, int scalar){
    Matrix <3,3> eye = {scalar,0,0,0,scalar,0,0,0,scalar};
    return eye*matrix;
}
Matrix<3,3> operator*(int scalar,Matrix<3,3> matrix){
    Matrix <3,3> eye = {scalar,0,0,0,scalar,0,0,0,scalar};
    return eye*matrix;
}
Matrix<3,3> operator*(Matrix<3,3> matrix, double scalar){
    Matrix <3,3> eye = {scalar,0,0,0,scalar,0,0,0,scalar};
    return eye*matrix;
}
Matrix<3,3> operator*(double scalar,Matrix<3,3> matrix){
    Matrix <3,3> eye = {scalar,0,0,0,scalar,0,0,0,scalar};
    return eye*matrix;
}

Matrix<3,3> Quaternion_To_Rotation_Matrix(Quaternion quat1){
    Matrix<3,3,float> eye3 = {1,0,0,
                              0,1,0,
                              0,0,1};
    Matrix<3,3,float> Rotation_Matrix;
    Matrix<1,1,float> norm_vec_sq = (~quat1.vec)*quat1.vec;

    Rotation_Matrix = eye3*(quat1.scalar*quat1.scalar-norm_vec_sq(0))+2*((quat1.vec)*(~quat1.vec))-2*quat1.scalar*skew3(quat1.vec);

    return Rotation_Matrix;
}

Matrix<4,4> skew4(Quaternion quat){
  Matrix<4,4> skew_matrix = {  quat.scalar, quat.vec(2),-quat.vec(1), quat.vec(0),
                              -quat.vec(2), quat.scalar, quat.vec(0), quat.vec(1),
                               quat.vec(1),-quat.vec(0),quat.scalar, quat.vec(2),
                              -quat.vec(0),-quat.vec(1),-quat.vec(2), quat.scalar
                            };
  return skew_matrix;
}


Matrix<3,3> skew3(Matrix<3,1,float> vector){
  Matrix<3,3> skew_matrix = {1,0,0,0,1,0,0,0,1};
  Matrix<3,1,float> aux_vec;
  for(int i = 0; i < 3; i++){
    RefMatrix<Matrix<3,3>,3,1> axis_i(skew_matrix.Submatrix<3,1>(0,i));
    aux_vec = axis_i;
    aux_vec = Cross(aux_vec,vector);
    axis_i = aux_vec*(float)(-1.0);
  }
  return  skew_matrix;
}


Matrix<3,1,float> Cross(Matrix<3,1,float>& matrix1,Matrix<3,1,float>& matrix2){
  Matrix<3,1,float> value;
  value(0) = matrix1(1)*matrix2(2) - matrix1(2)*matrix2(1);
  value(1) = matrix1(2)*matrix2(0) - matrix1(0)*matrix2(2);
  value(2) = matrix1(0)*matrix2(1) - matrix1(1)*matrix2(0);
  return value;
}


