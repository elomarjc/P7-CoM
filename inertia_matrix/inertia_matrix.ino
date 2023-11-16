#include <BasicLinearAlgebra.h>

using namespace BLA;

const int potPin_m1 = 35; // Motor 1
const int potPin_m5 = 34; // Motor 5
const int potPin_m6 = 39; // Motor 6

const float minPotValue = 4095.0;  // Minimum potentiometer value (leftmost)

float getPotentiometer_m1() {
  float potVoltage1 = analogRead(potPin_m1) * (3.3 / minPotValue);
  return calculateVoltageToPosition_m1(potVoltage1);
}
float getPotentiometer_m5() {
  float potVoltage5 = analogRead(potPin_m5) * (3.3 / minPotValue);
  return calculateVoltageToPosition_m5(potVoltage5);
}
float getPotentiometer_m6() {
  float potVoltage6 = analogRead(potPin_m6) * (3.3 / minPotValue);
  return calculateVoltageToPosition_m6(potVoltage6);
}

float calculateVoltageToPosition_m1(float voltage) {
  // Calculate the position within the original range
  return 0.8041 * voltage * voltage * voltage - 5.0581 * voltage * voltage + 10.821 * voltage + 0.5937 - 5;
}

// Motor 5
float calculateVoltageToPosition_m5(float voltage) {
  return 0.2521 * voltage * voltage * voltage - 1.1326 * voltage * voltage + 3.7706 * voltage + 0.0422 - 5;
}

// Motor 6
float calculateVoltageToPosition_m6(float voltage) {
  return -0.3239 * voltage  * voltage  * voltage  * voltage  + 2.8829 * voltage  * voltage  * voltage - 8.9187 * voltage  * voltage  + 12.441 * voltage  + 0.809  - 5;
}


// Function to calculate inertia matrix for a motor
BLA::Matrix<3, 3, float> calculateInertiaMatrix(
  BLA::Matrix<3, 1, float>& A_M,
  BLA::Matrix<3, 1, float>& A_m,
  BLA::Matrix<3, 1, float>& S,
  BLA::Matrix<3, 3, float>& I_R,
  float M,
  float u) {

  // Updated calculations based on the potentiometer value and mass
  BLA::Matrix<3, 1> d1 = A_M - A_m;
  BLA::Matrix<1, 1> a1 = ~d1 * d1;
  float a = sqrt(a1(0, 0)) / 2.0;
  float t = (u / 2 * a) + (1 / 2);

  // Calculate R as a function of t
  BLA::Matrix<3, 1, float> R = A_m * (1 - t) + A_M * t;

  // Calculate d
  BLA::Matrix<3, 1, float> d = R - S;

  // Calculate I_S
  BLA::Matrix<3, 3> aux = {M, 0, 0, 0, M, 0, 0, 0, M};
  BLA::Matrix<3, 3, float> I_S = I_R - aux * d * ~d;

  return I_S;
}

// Function to combine inertia matrices for three motors
BLA::Matrix<3, 3, float> combineInertiaMatrices(BLA::Matrix<3, 1, float>& u) {

  // Set S
  BLA::Matrix<3, 1, float> S = {0, 0, 0};

  // Motor 1 parameters
  BLA::Matrix<3, 1, float> A_M_m1 = {5, 0, 0};
  BLA::Matrix<3, 1, float> A_m_m1 = { -5, 0, 0};
  BLA::Matrix<3, 3, float> I_R_m1 = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float M_m1 = 0.06;
  float u_m1 = u(0,0);

  // Motor 5 parameters
  BLA::Matrix<3, 1, float> A_M_m5 = {0, 5, 0};
  BLA::Matrix<3, 1, float> A_m_m5 = {0, -5, 0};
  BLA::Matrix<3, 3, float> I_R_m5 = {2, 0, 0, 0, 2, 0, 0, 0, 2};
  float M_m5 = 0.06;
  float u_m5 = u(1,0);

  // Motor 6 parameters
  BLA::Matrix<3, 1, float> A_M_m6 = {0, 0, 5};
  BLA::Matrix<3, 1, float> A_m_m6 = {0, 0, -5};
  BLA::Matrix<3, 3, float> I_R_m6 = {3, 0, 0, 0, 3, 0, 0, 0, 3};
  float M_m6 = 0.06;
  float u_m6 = u(2,0);

  // Combine inertia matrices
  BLA::Matrix<3, 3, float> I_S_m1 = calculateInertiaMatrix(A_M_m1, A_m_m1, S, I_R_m1, M_m1, u_m1);
  BLA::Matrix<3, 3, float> I_S_m5 = calculateInertiaMatrix(A_M_m5, A_m_m5, S, I_R_m5, M_m5, u_m5);
  BLA::Matrix<3, 3, float> I_S_m6 = calculateInertiaMatrix(A_M_m6, A_m_m6, S, I_R_m6, M_m6, u_m6);

  return I_S_m1 + I_S_m5 + I_S_m6;
}

void setup() {
  // Serial communication setup
  Serial.begin(115200);
}

void loop() {
  // Combine inertia matrices
  BLA::Matrix<3, 1, float> u = {getPotentiometer_m1(), getPotentiometer_m5(), getPotentiometer_m6()};
  BLA::Matrix<3, 3, float> I_S_total = combineInertiaMatrices(u);

  // Display results
  Serial << "Total Inertia Matrix:\n" << I_S_total;

  delay(100); // Add a delay for stability
}
