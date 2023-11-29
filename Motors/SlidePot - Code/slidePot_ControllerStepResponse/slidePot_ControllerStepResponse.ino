#include "Arduino.h"

#define F 200

// Define controller parameters as constants
const float PI_KP = 2.4382f;
const float PI_KI = 0.73146f;
const float PI_LIM_MIN = -7.0f;
const float PI_LIM_MAX = 7.0f;
const float PI_LIM_MIN_INT = -5.0f;
const float PI_LIM_MAX_INT = 5.0f;
const float SAMPLE_TIME_S = 1 / F;

class PIController {
public:
  PIController()
    : Kp(PI_KP), Ki(PI_KI), T(SAMPLE_TIME_S), limMin(PI_LIM_MIN), limMax(PI_LIM_MAX), limMinInt(PI_LIM_MIN_INT), limMaxInt(PI_LIM_MAX_INT) {
    Init();
  }

  void Init() {
    // Clear controller
    integrator = 0.0f;
    prevError = 0.0f;
    out = 0.0f;
  }

  float Update(float setpoint, float measurement) {
    float error = setpoint - measurement;

    float proportional = Kp * error;

    integrator = integrator + 0.5f * Ki * T * (error + prevError);

    // Anti-wind-up via dynamic integrator clamping
    if (integrator > limMaxInt) {
      integrator = limMaxInt;
    } else if (integrator < limMinInt) {
      integrator = limMinInt;
    }

    // Compute output and apply limits
    out = proportional + integrator;

    if (out > limMax) {
      out = limMax;
    } else if (out < limMin) {
      out = limMin;
    }

    // Store error for later use
    prevError = error;

    // Return controller output
    return out;
  }

private:
  float Kp;          // Proportional gain
  float Ki;          // Integral gain
  float T;           // Sample time
  float integrator;  // Integral term
  float prevError;   // Previous error
  float out;         // Controller output
  float limMin;      // Min output limit
  float limMax;      // Max output limit
  float limMinInt;   // Min integrator limit
  float limMaxInt;   // Max integrator limit
};

PIController myController;

// Pins 36, 6, 7, 8 DEAD
// Motor 1
const int potPin_m1 = 35;  // 35 brown
const int in1_m1 = 2; // 2 white
const int in2_m1 = 15; // 15 yellow
const int enA_m1 = 4; // 4 green

// Motor 5
const int potPin_m5 = 34; // 34 brown
const int in1_m5 = 12; // 12 blue
const int in2_m5 = 14; // 14 purple
const int enA_m5 = 0; // 0 green

// Motor 6
const int potPin_m6 = 39; //39 brown
const int in1_m6 = 27; //16 white
const int in2_m6 = 26; //17 gray
const int enA_m6 = 13; //13 yellow

// Setting PWM properties
const int pwmChannel_m1 = 0;
const int pwmChannel_m5 = 1;
const int pwmChannel_m6 = 2;
const int resolution = 8;
int dutyCycle_m1 = 0;  // Initialize dutyCycle to 0
int dutyCycle_m5 = 0;
int dutyCycle_m6 = 0;

// Define the range of potentiometer values for leftmost and rightmost positions
const float minPotValue = 4095.0;  // Minimum potentiometer value (leftmost)
const int maxPotValue = 0;     // Maximum potentiometer value (rightmost)

// Target position
float targetPositionNow_m1 = 0;
float targetPositionNow_m5 = 0;
float targetPositionNow_m6 = 0;

// Time variables
unsigned long startTime = 0;
unsigned long currentTime = 0;

// Dead zone variables
static float dead_zone_m1;
static float dead_zone_m5;
static float dead_zone_m6;

// Function to calculate the voltage from the position (cm)
//float calculatePositionToVoltage(float position) {
//  return 0.0052 * position * position * position - 0.0288 * position * position + 0.1114 * position - 0.0161;
//}

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
  return 0.814 * voltage * voltage * voltage - 4.9125 * voltage * voltage + 10.123 * voltage + 0.9785 - 5;
}

void setVoltageMotor(float motorVoltage, float dz_correction, int in1, int in2, int pwmChannel, int dutyCycle) {
  if (motorVoltage > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (motorVoltage < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    // Stop the motor if the target position is reached
    ledcWrite(pwmChannel, LOW);
  }
  // Map motorVoltage to the duty cycle range [0, 255]
  dutyCycle = map(constrain(abs(motorVoltage) + dz_correction, 0, 10), 0, 10, 0, 255);
  ledcWrite(pwmChannel, dutyCycle);
}

float getPotentiometer_m1() {
  float potVoltage = analogRead(potPin_m1) * (3.3 / minPotValue);
  return calculateVoltageToPosition_m1(potVoltage);
}
float getPotentiometer_m5() {
  float potVoltage = analogRead(potPin_m5)* (3.3 / minPotValue);
  return calculateVoltageToPosition_m5(potVoltage);
}
float getPotentiometer_m6() {
  float potVoltage = analogRead(potPin_m6) * (3.3 / minPotValue);
  return calculateVoltageToPosition_m6(potVoltage);
}


void setup() {
  // Motor 1
  pinMode(potPin_m1, INPUT);
  pinMode(enA_m1, OUTPUT);
  pinMode(in1_m1, OUTPUT);
  pinMode(in2_m1, OUTPUT);
  // Motor 5
  pinMode(potPin_m5, INPUT);
  pinMode(enA_m5, OUTPUT);
  pinMode(in1_m5, OUTPUT);
  pinMode(in2_m5, OUTPUT);
  // Motor 6
  pinMode(potPin_m6, INPUT);
  pinMode(enA_m6, OUTPUT);
  pinMode(in1_m6, OUTPUT);
  pinMode(in2_m6, OUTPUT);

  // Configure LED PWM functionality
  ledcSetup(pwmChannel_m1, 30000, resolution);
  ledcSetup(pwmChannel_m5, 30000, resolution);
  ledcSetup(pwmChannel_m6, 30000, resolution);

  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(enA_m1, pwmChannel_m1);
  ledcAttachPin(enA_m5, pwmChannel_m5);
  ledcAttachPin(enA_m6, pwmChannel_m6);

  Serial.begin(115200);
  while (!Serial)
    ;

  delay(1000);  // Wait for 1 second before starting the step response

  myController.Init();  // Initialize the controller

  // Record the start time
  startTime = micros();
}

void loop() {
  // Calculate the elapsed time
  currentTime = micros();
  unsigned long elapsedTime = (currentTime - startTime);  // Convert to seconds

  // Step response test: Change the target position after a certain time
  if (elapsedTime < 2 * 1000000) {  // For the first 2 seconds, maintain the initial target
    targetPositionNow_m1 = -3.0;
    targetPositionNow_m5 = -3.0;
    targetPositionNow_m6 = -3.0;
  } else {
    targetPositionNow_m1 = 3.0;
    targetPositionNow_m5 = 3.0;
    targetPositionNow_m6 = 3.0;
  }
  float input_m1 = getPotentiometer_m1();
  float output_m1 = myController.Update(targetPositionNow_m1, input_m1);
  if (abs(targetPositionNow_m1 - input_m1) <= 0.03) {
    dead_zone_m1 = 0;
  } else {
    dead_zone_m1 = 2.4;
  }
  setVoltageMotor(-output_m1, dead_zone_m1, in1_m1, in2_m1, pwmChannel_m1, dutyCycle_m1);  // 5 volts

  float input_m5 = getPotentiometer_m5();
  float output_m5 = myController.Update(targetPositionNow_m5, input_m5);
  if (abs(targetPositionNow_m5 - input_m5) <= 0.03) {
    dead_zone_m5 = 0;
  } else {
    dead_zone_m5 = 2.4;
  }
  setVoltageMotor(-output_m5, dead_zone_m5, in1_m5, in2_m5, pwmChannel_m5, dutyCycle_m5);  // 5 volts

  float input_m6 = getPotentiometer_m6();
  float output_m6 = myController.Update(targetPositionNow_m6, input_m6);
  if (abs(targetPositionNow_m6 - input_m6) <= 0.03) {
    dead_zone_m6 = 0;
  } else {
    dead_zone_m6 = 2.4;
  }
  setVoltageMotor(-output_m6, dead_zone_m6, in1_m6, in2_m6, pwmChannel_m6, dutyCycle_m6);  // 5 volts

  if (elapsedTime >= 2 * 1000000) {
    Serial.print("Time: ");
    Serial.print(elapsedTime / 1000.0 - 2000);  // Time in seconds
    Serial.print('\t');
    Serial.print("Pot 1: ");
    Serial.print(getPotentiometer_m1());
    Serial.print('\t');
    Serial.print("Out 1: ");
    Serial.print(output_m1);
    Serial.print('\t');
    Serial.print("Pot 5: ");
    Serial.print(getPotentiometer_m5());
    Serial.print('\t');
    Serial.print("Out 5: ");
    Serial.print(output_m5);
    Serial.print('\t');
    Serial.print("Pot 6: ");
    Serial.print(getPotentiometer_m6());
    Serial.print('\t');
    Serial.print("Out 6: ");
    Serial.println(output_m6);
  }

  bool time_exceeded = true;
  while (micros() - currentTime < 1000000 / F) {
    time_exceeded = false;
  }
/*
  if (time_exceeded) {
    Serial.print("TIME EXCEEDED: ");
    Serial.print((micros() - currentTime) / 1000.0);
    Serial.println("ms.");
  }
*/
  if (elapsedTime > 10 * 1000000) {  // After 4 seconds, turn off the motor
    ledcWrite(pwmChannel_m1, LOW);
    ledcWrite(pwmChannel_m5, LOW);
    ledcWrite(pwmChannel_m6, LOW);
    while (true)
      ;
  }
}
