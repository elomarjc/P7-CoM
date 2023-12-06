#include "CoM_Estimation.h"


// TBD:
// -CoMestimation


Matrix<3,1,float> Backwards_Difference_Equation(Matrix<3,1,float> new_value);
Matrix<3,1,float> Torque_estimation(Matrix<3,1,float> acc_est, Matrix<3,1,float> w_est,Matrix<3,1,float> weights_dist);
Matrix<3,1,float> CoM_estimation(Quaternion q_est, Matrix<3,1,float>N_est);
Quaternion quat_delay(Quaternion q_est, int delay);

Matrix<3, 3, float> calculateInertiaMatrix(Matrix<3, 1, float>& A_M, Matrix<3, 1, float>& A_m, Matrix<3, 1, float>& S, Matrix<3, 3, float>& I_R,float M,float u);
Matrix<3, 3, float> combineInertiaMatrices(Matrix<3, 1, float>& u);

Filter ang_accel_filter[3];

class{
  public:
  Matrix<3,1,float> acc_est;
  Matrix<3,1,float> N_est;
  Matrix<3,1,float> CoM_est;
}Output;


float dt_CoM = 0;
float M = 1;
// Inits

void CoM_Estimation_Innit(float dt_in,float M_in){
  dt_CoM = dt_in;
  M = M_in;
  // Butter filter of order 4, at 50Hz and cutoff 10Hz
  int size = 4;
  float numerator[] = {0,5.9995/100000,5.9356/10000,5.3466/10000,4.3847/100000};
  float denominator[] = {1,-3.4788,4.5680,-2.6809,0.5930};
  for (int i = 0; i < 3 ; i++){
   ang_accel_filter[i].Initialize(numerator,denominator,size);
  }
  
}

Matrix<3,1,float> CoM_estimation_GET_acc_est(){
  return Output.acc_est;
}
Matrix<3,1,float> CoM_estimation_GET_N_est(){
  return Output.N_est;
}
Matrix<3,1,float> CoM_estimation_GET_CoM_est(){
  return Output.CoM_est;
}
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void CoM_estimation_Update(Quaternion q_est, Matrix<3,1,float> w_est,Matrix<3,1,float>weights_pos){
  // Compute acc est
  Matrix<3,1,float> acc_est = Backwards_Difference_Equation(w_est);
  // Filter acc est
  for(int i = 0; i < 3; i++){
    acc_est(i) = ang_accel_filter[i].Update(acc_est(i));
  }
  // Compute N est
  Matrix<3,1,float> N_est = Torque_estimation(acc_est,w_est,weights_pos);
  // Compute CoM est

  Quaternion q_est_delayed = quat_delay(q_est,10);
  Matrix<3,1,float> CoM_est = CoM_estimation(q_est,N_est);
  
  // Store values
  Output.acc_est = acc_est;
  Output.N_est = N_est;
  Output.CoM_est = CoM_est;
}
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Quaternion quat_delay(Quaternion q_est, int delay){
  static Quaternion Quaternion_list[40];

  for(int i = delay-1; i >0;i--){
    Quaternion_list[i] = Quaternion_list[i-1];
  }
  Quaternion_list[0] = q_est;
  return Quaternion_list[delay-1];
}

Matrix<3,1,float> Backwards_Difference_Equation(Matrix<3,1,float> new_value){
  static Matrix<3,1,float> prev_value = {0,0,0};
  static Matrix<3,3,float> eye_dt = {dt_CoM,0,0,0,dt_CoM,0,0,0,dt_CoM};
  Matrix<3,1,float> acc_est = eye_dt*(new_value-prev_value);

  prev_value = new_value;
  return acc_est;
}
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Matrix<3,1,float> Torque_estimation(Matrix<3,1,float> acc_est, Matrix<3,1,float> w_est,Matrix<3,1,float> weights_dist){
  // Compute new Inertia matrix
  Matrix<3,3,float> Inertia_Matrix = combineInertiaMatrices(weights_dist);
  // Solve equation
  Matrix<3,1,float> L = Inertia_Matrix*w_est;
  Matrix<3,1,float> N_B_effect = Cross(w_est,L);
  Matrix<3,1,float> N_est = Inertia_Matrix*acc_est - N_B_effect;
  return N_est;
}
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Matrix<3,1,float> CoM_estimation(Quaternion q_est, Matrix<3,1,float>N_est){
  static Matrix<3,1,float> CoM_est = {0,0,0};

  static Matrix<3,3,float> P = {1,0,0,0,1,0,0,0,1};
  static Matrix<3,3,float> F = {1,0,0,0,1,0,0,0,1};
  static Matrix<3,3,float> Q = {0.01,0,0,0,0.01,0,0,0,0.01};
  static Matrix<3,3,float> R = {1,0,0,0,1,0,0,0,1};


  static Matrix<3,3,float> H = {0,0,0,0,0,0,0,0,0};
  static Matrix<3,1,float> z = {0,0,0};
  static Matrix<3,3,float> K = {0,0,0,0,0,0,0,0,0};

  // Update equations
  // CoM_est = CoM_est static dynamics
  P = F*P*(~F)+Q;

  

  // Observer equations
  static Matrix<3,3,float> Attitude_Matrix = Quaternion_To_Rotation_Matrix(q_est);
  static Matrix<3,3,float> M_eye = {M,0,0,0,M,0,0,0,M};
  static Matrix<3,1,float> g = {0,0,-9.81};
  
  H = -skew3(Attitude_Matrix*g)*M_eye;
  K =P*(~H)*Inverse(H*P*(~H)+R);

  CoM_est = CoM_est + K*(N_est-H*CoM_est);
  
  P = P - K*H*P;

  return CoM_est;
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Matrix<3,3,float> calculateInertiaMatrix(Matrix<3, 1, float>& A_M,Matrix<3, 1, float>& A_m,Matrix<3, 1, float>& S,Matrix<3, 3, float>& I_R,float M,float u) {

  // Updated calculations based on the potentiometer value and mass
  Matrix<3, 1> d1 = A_M - A_m;
  Matrix<1, 1> a1 = ~d1 * d1;
  float a = sqrt(a1(0, 0)) / 2.0;
  float t = (u / 2.0 * a) + 0.5;

  // Calculate R as a function of t
  Matrix<3, 1, float> R = A_m * (1 - t) + A_M * t;

  // Calculate d
  Matrix<3, 1, float> d = R - S;

  // Calculate I_S
  Matrix<3, 3> aux = {M, 0, 0, 0, M, 0, 0, 0, M};
  Matrix<3, 3, float> I_S = I_R - aux * d * ~d;

  return I_S;
}

// Function to combine inertia matrices for three motors
Matrix<3,3,float> combineInertiaMatrices(Matrix<3, 1, float>& u) {

  // Set S
  Matrix<3, 1, float> S = {0, 0, 0};

  // Motor 1 parameters
  Matrix<3, 1, float> A_M_m1 = {5, 0, 0};
  Matrix<3, 1, float> A_m_m1 = { -5, 0, 0};
  Matrix<3, 3, float> I_R_m1 = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float M_m1 = 0.06;
  float u_m1 = u(0,0);

  // Motor 5 parameters
  Matrix<3, 1, float> A_M_m5 = {0, 5, 0};
  Matrix<3, 1, float> A_m_m5 = {0, -5, 0};
  Matrix<3, 3, float> I_R_m5 = {2, 0, 0, 0, 2, 0, 0, 0, 2};
  float M_m5 = 0.06;
  float u_m5 = u(1,0);

  // Motor 6 parameters
  Matrix<3, 1, float> A_M_m6 = {0, 0, 5};
  Matrix<3, 1, float> A_m_m6 = {0, 0, -5};
  Matrix<3, 3, float> I_R_m6 = {3, 0, 0, 0, 3, 0, 0, 0, 3};
  float M_m6 = 0.06;
  float u_m6 = u(2,0);

  // Combine inertia matrices
  Matrix<3, 3, float> I_S_m1 = calculateInertiaMatrix(A_M_m1, A_m_m1, S, I_R_m1, M_m1, u_m1);
  Matrix<3, 3, float> I_S_m5 = calculateInertiaMatrix(A_M_m5, A_m_m5, S, I_R_m5, M_m5, u_m5);
  Matrix<3, 3, float> I_S_m6 = calculateInertiaMatrix(A_M_m6, A_m_m6, S, I_R_m6, M_m6, u_m6);

  return I_S_m1 + I_S_m5 + I_S_m6;
}
