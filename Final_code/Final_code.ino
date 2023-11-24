// IMU pins
#define SDA_PIN  0 // IMU
#define SCL_PIN  1 // IMU
// Motor 1
#define POT_PIN_M1 = 35
#define ENA_M1 = 4
#define IN1_M1 = 2
#define IN2_M1 = 15
#define PWM_M1 = 0
// Motor 5
#define POT_PIN_M5 = 34
#define ENA_M5 = 0
#define IN1_M5 = 12
#define IN2_M5 = 14
#define PWM_M1 = 1
// Motor 6
#define POT_PIN_M6 = 39
#define ENA_M6 = 13
#define IN1_M6 = 27
#define IN2_M6 = 26
#define PWM_M1 = 2



#define PI 3.141592
#define F 20
float dt = 1.0/F;
#define MILLION pow(10,6)

#include "Sensor.h"
#include "LKF.h"
#include "Pot_Control.h"

long int start;

Sensor sensor;
LKFType LKF;
POT Pot1;
PIController Controller1;
POT Pot5;
PIController Controller5;
POT Pot6;
PIController Controller6;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  sensor.Initialize(SDA_PIN,SCL_PIN);
  Pot1.Initialize(POT_PIN_M1, ENA_M1, IN1_M1, IN2_M1, PWM_M1);
  Pot1.InitPotFunction(0.8041, 5.0581, 10.821, 0.5937);
  Controller1.Initialize();
  Pot5.Initialize(POT_PIN_M5, ENA_M5, IN1_M5, IN2_M5, PWM_M5);
  Pot5.InitPotFunction(0.2521, 1.1326, 3.7706, 0.0422);
  Controller5.Initialize();
  Pot6.Initialize(POT_PIN_M6, ENA_M6, IN1_M6, IN2_M6, PWM_M6);
  Pot6.InitPotFunction(0.814, 4.9125, 10.123, 0.9785);
  Controller6.Initialize();


  sensor.Calibrate_magnetometer();

}

void loop() {
  start = micros();
  // Update measurements
  sensor.Update();


}
