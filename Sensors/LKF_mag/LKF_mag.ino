#define SDA_PIN  6
#define SCL_PIN  5
#define PI 3.141592
#define F 20
float dt = 1.0/F;
#define MILLION pow(10,6)


#include "LKF.h"
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
  Serial.print(sensor.magn_meas(2));
  Serial.print(",");
  //Serial.println();
  Serial.print("Angle x,z:");
  Serial.println(1/PI*180*acos(sensor.Dot_Product(sensor.axis_x_mag, sensor.axis_z_accel)));
  //sensor.Plotter_Euler_Angles();
  //Serial.println();
  /**
  Serial.print(",Phi_KF:");
  Serial.print(LKF.x_vector_clamped(0));
  Serial.print(",Theta_KF:");
  Serial.print(LKF.x_vector_clamped(1));
  Serial.print(",Psi_KF:");
  Serial.print(LKF.x_vector_clamped(2));
  Serial.println();
  **/
  // Time management
  if(((float)(micros()-start))>1.0/F*MILLION){
    Serial.println("TIME EXCEEEEEEEEEEEEEEEEEEEEEEEEEDED");
  }
  while(((float)(micros()-start))<1.0/F*MILLION);
}



// JUNK --------------------------------------------------------

