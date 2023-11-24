#ifndef POT_CONTROL
#define POT_CONTROL
#include "Arduino.h"

class POT{
    public:
    // Must be executed in setup, connects to potentiometer
    void Initialize(int potPin_Int, int enA_Int, int in1_Int, int in2_Int, int PWMpin_Int);
    // Controls the motor
    void setVoltageMotor(float motorVoltage, float dz_correction);
    // Define the values in the potentiometer function
    void InitPotFunction(float fourth_Int, float thrid_Int, float second_Int, float first_Int);
    // Calculates position on the potentiometer fra voltage
    float VoltageToPosition(float voltage);
    // Read the potentiometer voltage
    float PotVoltage();

    private:
    // Variables
    const float minPotValue = 4095.0; // Minimum potentiometer value (leftmost)
    const int maxPotValue = 0; // Maximum potentiometer value (rightmost)
    const int resolution = 8;
    int Potpin; // Potentiometer pin
    int enA; // Enable pin
    int in1; // First servo track pin 
    int in2; // Second servo track pin
    int PWMpin; // PWM pin
    float first; // First order function value
    float second; // Second order function value
    float third; // Thrid order function value
    float fourth; // Fourth order function value
};


class PIController {
public:
  // Resets/Initializes the values in the controller
 void Initialize();
 // Updates controller value
 float Update(float setpoint, float measurement);
private:
  float Kp = 2.4382;          // Proportional gain
  float Ki = 0.73146;          // Integral gain
  float T = 1/200;           // Sample time
  float integrator;  // Integral term
  float prevError;   // Previous error
  float out;         // Controller output
  float limMin = -7.0;      // Min output limit
  float limMax = 7.0;      // Max output limit
  float limMinInt = -5.0;   // Min integrator limit
  float limMaxInt = 5.0;   // Max integrator limit
};

#endif
