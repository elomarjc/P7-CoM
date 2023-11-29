#ifndef LKF_H
#define LKF_H

// Initialize LKF: define functions
// Run LKF: update
// Print out values
// Encapsuling needed? -> No, system should be able to change on the fly, so EKF can be performed with a LKF.
// Inheritance? -> Yes! It must be able to be inherited for the EKF
// Customizable: Use accel,gyro,magnetorquer, etc, etc, etc
// Add tuning possibilities
// Must be indep from sensors, but capable of pooling directly from them

#include <BasicLinearAlgebra.h>
using namespace BLA;

extern float dt;

class LKFType{
  public:

  BLA::Matrix<3,3,float> f_var_matrix = {10*0.0140*dt,0,0,0,10*0.0172*dt,0,0,0,100*dt}; // NOTE: this is the var of alpha,phi& sigma, NOT of fx,fy,fz
  BLA::Matrix<3,1,float> gyro_mean = {-0.8630,-0.4062,-0.1584};                         //NOTE: Values are negated for the gyro correction
  BLA::Matrix<3,3,float> g_var_matrix = {0.0150*dt,0,0,0,0.0163*dt,0,0,0,0.0127*dt};
  BLA::Matrix<3,3,float> bias_matrix = {0.001*dt,0,0,0,0.001*dt,0,0,0,0.001*dt};
  BLA::Matrix<6,1,float> x_vector = gyro_mean&&O_vector;
  BLA::Matrix<6,3,float> Kalman_gain = {0,0,0}; 

  BLA::Matrix<6,6,float> F_matrix ={1,  0,  0,-dt,  0,  0,
                                           0,  1,  0,  0,-dt,  0,
                                           0,  0,  1,  0,  0,-dt,
                                           0,  0,  0,  1,  0,  0,
                                           0,  0,  0,  0,  1,  0,
                                           0,  0,  0,  0,  0,  1};
  BLA::Matrix<6,3,float> B_matrix = {dt,  0,  0,
                                             0, dt,  0,
                                             0,  0, dt,
                                             0,  0,  0,
                                             0,  0,  0,
                                             0,  0,  0};
  BLA::Matrix<3,6,float> H_matrix = {1,0,0,0,0,0,
                                            0,1,0,0,0,0,
                                            0,0,1,0,0,0};
  BLA::Matrix<6,6,float> P_matrix = (g_var_matrix||O_matrix)&&(O_matrix||O_matrix);    
   
  BLA::Matrix<3,3,float> R_matrix = f_var_matrix;  // Measurement noise variance
  BLA::Matrix<6,6,float> Q_matrix = (g_var_matrix||O_matrix)&&(O_matrix||bias_matrix);  //    System   noise variance
  void Update(BLA::Matrix<3, 1, float>&   eul_ang_a, BLA::Matrix<3, 1, float>&   gyro_w);

  BLA::Matrix<6,1,float> x_vector_clamped = {0,0,0,0,0,0};
  private:

  BLA::Matrix<3,1,float> O_vector = {0,0,0};
  BLA::Matrix<3,3,float> O_matrix = {0,0,0,0,0,0,0,0,0};
  void Clamp_Principal_Values();
  void updateFilter(BLA::Matrix<3, 1, float>&   eul_ang_a, BLA::Matrix<3, 1, float>&   gyro_w);
};
/**
void LKFType::Set_System_Equations(){

}

void LKFType::Set_InitialState(){

}

void LKFType::Set_Variance_Input(){

}

void LKFType::Set_Variance_Output(){

}
**/

void LKFType::Update(BLA::Matrix<3, 1, float>&   eul_ang_a, BLA::Matrix<3, 1, float>&   gyro_w){
  this->updateFilter(eul_ang_a,gyro_w);
  this->Clamp_Principal_Values();
}

void LKFType::Clamp_Principal_Values(){
  x_vector_clamped = x_vector;
  // Limiting angles to principal values
  static float limits[3] = {180,90,180};
  for(int i = 0; i < 3; i++){
    // Case angle > maximum
    if(x_vector_clamped(i)>limits[i]){
    x_vector_clamped(i)-= limits[i]*2;
    }
    // Case angle < minimum
    else if(x_vector_clamped(i)<-limits[i]){
      x_vector_clamped(i)+=limits[i]*2;
    }
  }
}

void LKFType::updateFilter(BLA::Matrix<3, 1, float>&   eul_ang_a, BLA::Matrix<3, 1, float>&   gyro_w){
  // Needed definitions

  // State x already defined: x_vector

  x_vector = F_matrix*x_vector+B_matrix*gyro_w;      // predict the next value
  P_matrix = F_matrix*P_matrix*(~F_matrix)+Q_matrix; // Predict the correlation variance

  Kalman_gain = P_matrix*(~H_matrix)*Inverse(H_matrix*P_matrix*(~H_matrix)+R_matrix); // Compute new kalman gain
  x_vector = x_vector + Kalman_gain*(eul_ang_a-H_matrix*x_vector);                    // Correct the estimation with new value
  P_matrix = P_matrix - Kalman_gain*H_matrix*P_matrix;                                // Correct the state correlation matrix

}

#endif