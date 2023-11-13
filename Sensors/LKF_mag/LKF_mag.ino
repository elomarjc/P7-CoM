#define SDA_PIN  6
#define SCL_PIN  5
#define PI 3.141592
#define F 20
float dt = 1.0/F;
#define MILLION pow(10,6)


#include "Sensor.h"

long int start;

Sensor sensor;
LKFType LKF;

void setup(){
  Serial.begin(115200);
  while (!Serial);
  sensor.Initialize(SDA_PIN,SCL_PIN);
}



void loop(){
  start = micros();

  // Update measurements
  sensor.Update();
  //LKF.Update(sensor.Euler_angles_accelerometer,sensor.gyro_meas);

  //sensor.Plotter_Accel_Filtered();

  
  Serial.print("mag_X:");
  Serial.print(sensor.magn_meas(0));
  Serial.print(",mag_Y:");
  Serial.print(sensor.magn_meas(1));
  Serial.print(",mag_Z:");
  Serial.println(sensor.magn_meas(2));

  // Time management
  if(((float)(micros()-start))>1.0/F*MILLION){
    Serial.println("TIME EXCEEEEEEEEEEEEEEEEEEEEEEEEEDED");
  }
  while(((float)(micros()-start))<1.0/F*MILLION);
}



// JUNK --------------------------------------------------------

