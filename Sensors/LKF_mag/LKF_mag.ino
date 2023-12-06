#define SDA_PIN  0
#define SCL_PIN  1
#define Freq 20   //change F in "FinalCode.ino" to Freq, so no calling problems with un-qualified ID's
float dt = 1.0/Freq;
#define MILLION pow(10,6)

#include "Sensor.h"
#include "LKF.h"

// Magnetic Declination for Aalborg
float mag_dec1 = 4.11;

long int start;

Sensor sensor;
LKFType LKF;


void setup(){
  Serial.begin(115200);
  while (!Serial);
  sensor.Initialize(SDA_PIN,SCL_PIN);
  sensor.Calibrate_magnetometer();
}

void loop(){
  start = micros();

  // Update measurements
  sensor.Update();
  //LKF.Update(sensor.Euler_angles_accelerometer,sensor.gyro_meas);
  //sensor.Plotter_Accel_Filtered();
// Calculate angle for heading, assuming board is parallel to the ground and +Y points towards heading
double heading = -1 * atan2(sensor.magn_meas(0), sensor.magn_meas(1)) * 180 / PI;
// Apply magnetic declination to convert magnetic heading to geographic heading
  heading += mag_dec1;
  // Convert heading to 0...360 degrees
  if(heading < 0){
    heading += 360;
  }

  Serial.print(sensor.magn_meas(0));
  Serial.print(",");
  Serial.print(sensor.magn_meas(1));
  Serial.print(",");
  Serial.print(sensor.magn_meas(2));
  //Serial.print(",heading:");
  //Serial.println(heading);
  Serial.println();

  // Time management
  if(((float)(micros()-start))>1.0/Freq*MILLION){
    Serial.println("TIME EXCEEEEEEEEEEEEEEEEEEEEEEEEEDED");
  }
  while(((float)(micros()-start))<1.0/Freq*MILLION);
}
