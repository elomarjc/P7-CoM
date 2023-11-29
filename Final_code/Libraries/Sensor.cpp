#include "Sensor.h"

void Sensor::Initialize(int pin_SDA,int pin_scl){
    // First, start the connection with the IMU, requires SDA and SCL pins
    Wire.begin(pin_SDA, pin_scl);  
    gatherer.setWire(&Wire);

    // Initialize all 3 measurements
    gatherer.beginAccel();
    gatherer.beginGyro();
    gatherer.beginMag();  

    // Set up Accel filter 


    // Right now butter 3rd order, 15Hz cut-off wiht Fs = 20Hz
    //float num[4] = {0,0.04765,0.1291,0.02251};
    //float den[4] = {0,1.567,-0.9895,0.2231};
    // Right now butter 3rd order, 2Hz cut-off wiht Fs = 20Hz
    float num[4] = {0,0.0299,0.08648,0.01595};
    float den[4] = {0,1.783,-1.2 ,0.2846};
    for(int i = 0; i < 3;i++){
      
      accel_filter[i].Initialize(num,den,4);
    }
}

// Iterate through the arrays to find max and min values

  void Sensor::get_Max_Min(float* samples, int size, float* max, float* min){
    *max = samples[0];
    *min = samples[0];
    for (int i = 0; i < size; i++) {
        // Compare elements from all arrays
        if (samples[i] > *max) {
            *max = samples[i];
        }
        if (samples[i] < *min) {
            *min = samples[i];
        }
   }
  }

// ------------------------------------------------------------------------------------------------------------------------------------------------------

void Sensor::Calibrate_magnetometer(){
	Serial.println("Starting calibration, move the device around all axis for at least 30 seconds");
  int test_duration_ms = 30000;
  int sample_delay = 100;
	Serial.print("expected array size:");
  Serial.println(test_duration_ms / sample_delay);
	float x_values[300], y_values[300], z_values[300];
	unsigned int start_time = millis();
  for(int i = 0; i < test_duration_ms / sample_delay; i++)
	{
    Update_Measurements();
		x_values[i] = magn_meas(0);
		y_values[i] = magn_meas(1);
		z_values[i] = magn_meas(2);
		delay(sample_delay);
	}
  int size = sizeof(x_values) / sizeof(x_values[0]);
  float max_X, min_X;
  get_Max_Min(x_values, size, &max_X, &min_X);
  float max_Y, min_Y;
  get_Max_Min(y_values, size, &max_Y, &min_Y);
  float max_Z, min_Z;
  get_Max_Min(z_values, size, &max_Z, &min_Z);
	float offset_x = (max_X + min_X) / 2;
	float offset_y = (max_Y + min_Y) / 2;
	float offset_z = (max_Z + min_Z) / 2;
	float avg_delta_x = (max_X - min_X) / 2;
	float avg_delta_y = (max_Y - min_Y) / 2;
	float avg_delta_z = (max_Z - min_Z) / 2;
	float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;
	float scale_x = (float)avg_delta / avg_delta_x;
	float scale_y = (float)avg_delta / avg_delta_y;
	float scale_z = (float)avg_delta / avg_delta_z;
	// Set Calibration
	magn_hard_iron = {offset_x, offset_y, offset_z};
	magn_soft_iron = {scale_x, scale_y, scale_z};
	Serial.println("Calibration has been set");
}

// Calculate angle for heading, assuming board is parallel to the ground and +Y points towards heading
  float Sensor::Heading(){
    float heading = -1 * atan(magn_meas(0), magn_meas(1)) * 180 / PI;
    // Apply magnetic declination to convert magnetic heading to geographic heading
    heading += mag_dec1;
    // Convert heading to 0...360 degrees
    if(heading < 0){
    heading += 360;
  }
  return heading;
  }


void Sensor::Update(){

  this->Update_Measurements();
  this->Offset_corrections();
  this->Filter_Measurements();
  this->Compute_axis();
  this->Compute_Euler_Angles();
}
void Sensor::Compute_axis(){
  axis_x_mag = Normalize(magn_meas_filtered);
  axis_z_accel = -Normalize(accel_meas_filtered);
  axis_y_mix = vector_product(axis_z_accel,axis_x_mag);
}
void Sensor::Offset_corrections(){
  accel_meas -= accel_offset;
  gyro_meas -= gyro_offset;
  magn_meas(0) = (magn_meas(0) - magn_hard_iron(0)) * magn_soft_iron(0);
  magn_meas(1) = (magn_meas(1) - magn_hard_iron(1)) * magn_soft_iron(1);
  magn_meas(2) = (magn_meas(2) - magn_hard_iron(2)) * magn_soft_iron(2);
  //magn_meas = Normalize(magn_meas);
}
void Sensor::Update_Measurements(){
  if (gatherer.accelUpdate() == 0) {
    accel_meas = {gatherer.accelX(),gatherer.accelY(),gatherer.accelZ()};
     
  }
  

  
  if (gatherer.gyroUpdate() == 0) {
    gyro_meas = {-gatherer.gyroX(),-gatherer.gyroY(),-gatherer.gyroZ()}; // Minus make it follow convention: angles increase counter clockwise
    
  }
  
  if (gatherer.magUpdate() == 0) {
    magn_meas = {gatherer.magX(),gatherer.magY(),gatherer.magZ()};
  }  


}
// ------------------------------------------------------------------------------------------------------------------------------------------------------

float Sensor::Dot_Product(BLA::Matrix<3,1,float> vec1, BLA::Matrix<3,1,float> vec2){
  return vec1(0)*vec2(0)+vec1(1)*vec2(1)+vec1(2)*vec2(2);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------
BLA::Matrix<3,1,float> Sensor::vector_product(BLA::Matrix<3,1,float> vec1, BLA::Matrix<3,1,float> vec2){
  BLA::Matrix<3,1,float> result = {vec1(1)*vec2(2)-vec1(2)*vec2(1),vec1(2)*vec2(0)-vec1(0)*vec2(2),vec1(0)*vec2(1)-vec1(1)*vec2(0)};
  return result;
} 
// ------------------------------------------------------------------------------------------------------------------------------------------------------

BLA::Matrix<3,1,float> Sensor::Normalize(BLA::Matrix<3,1,float>& vec_1){
    BLA::Matrix<3,1,float> vector_1 = vec_1;
    float size  = sqrt(vector_1(0)*vector_1(0)+vector_1(1)*vector_1(1)+vector_1(2)*vector_1(2));
    vector_1(0) /= size;
    vector_1(1) /= size;
    vector_1(2) /= size;
    return vector_1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------

void Sensor::Compute_Euler_Angles(){

  // Accel


  // Cumpute the euler angles
  Euler_angles_accelerometer(0) = 180/PI* atan2(axis_z_accel(1),axis_z_accel(2));
  //Euler_angles_accelerometer(1) = 180/PI* atan(-axis_z_accel(0)   /   ( axis_z_accel(1)*sin(Euler_angles_accelerometer(0)) + axis_z_accel(2)*cos(Euler_angles_accelerometer(0)) ))  ;
  Euler_angles_accelerometer(1) = 180/PI*asin(-axis_z_accel(0));
  // Remove the correction into principal angle, needed for better filtering
  
  static BLA::Matrix<3,1,float>  prev_values = Euler_angles_accelerometer;
  static int  made_laps[3] = {0,0,0};
  static int loop_number = 0;
  if(loop_number > 5){
    for (int i = 0; i < 3; i++){
      // If their sign does not match
      if(Euler_angles_accelerometer(i)*prev_values(i)<0){
        // If the values are not 0
        if(abs(Euler_angles_accelerometer(i))+abs(prev_values(i))>90){
          // A lap has occured: count it in
          if(Euler_angles_accelerometer(i)>prev_values(i)){
            // Jumped from negative to positive
            made_laps[i] -= 1;
          }else{
            // Jumped from positive to negative
            made_laps[i] += 1;
          }
        }
      }
    }
  }
  loop_number++;
  prev_values = Euler_angles_accelerometer;
  static float ang_per_lap[3] = {360,180,360};
  // Add laps to accel
  for(int i = 0; i < 3; i++){
    Euler_angles_accelerometer(i) += ang_per_lap[i]*made_laps[i]; 
  }
  
  

  // gyro
  BLA::Matrix<3,1,float> gyro_dt = {dt*gyro_meas(0),dt*gyro_meas(1),dt*gyro_meas(2)};
  Euler_angles_gyroscope += gyro_dt;

  // Limiting angles to principal values
  static float limits[3] = {180,90,180};
  for(int i = 0; i < 3; i++){
    // Case angle > maximum
    if(Euler_angles_gyroscope(i)>limits[i]){
    Euler_angles_gyroscope(i)-= limits[i]*2;
    }
    // Case angle < minimum
    else if(Euler_angles_gyroscope(i)<-limits[i]){
      Euler_angles_gyroscope(i)+=limits[i]*2;
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------

void Sensor::Filter_Measurements(){

  for(int i = 0; i < 3; i++){
      accel_meas_filtered(i) = accel_filter[i].Update(accel_meas(i));
  }
  gyro_meas_filtered = gyro_meas;
  magn_meas_filtered = magn_meas;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------

void Sensor::Plotter_Gyro_Angles(){  
  Serial.print("Phi_gyr:");
  Serial.print(this->Euler_angles_gyroscope(0));
  Serial.print(",");

  Serial.print("Theta_gyr:");
  Serial.print(this->Euler_angles_gyroscope(1));
  Serial.print(",");
  
  Serial.print("Psi_gyr:");
  Serial.print(this->Euler_angles_gyroscope(2));
  Serial.print(",");
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------

void Sensor::Plotter_Euler_Angles(){
  Serial.print("Phi_acc:");
  Serial.print(this->Euler_angles_accelerometer(0));
  Serial.print(",");

  Serial.print("Theta_acc:");
  Serial.print(this->Euler_angles_accelerometer(1));
  Serial.print(",");
  
  Serial.print("Psi_acc:");
  Serial.print(this->Euler_angles_accelerometer(2));
  Serial.print(",");
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------

void Sensor::Plotter_Accel_Filtered(){
  Serial.print("Fx_acc:");
  Serial.print(this->accel_meas_filtered(0)*180);
  Serial.print(",");

  Serial.print("Fy_acc:");
  Serial.print(this->accel_meas_filtered(1)*180);
  Serial.print(",");
  
  Serial.print("Fz_acc:");
  Serial.print(this->accel_meas_filtered(2)*180);
  Serial.print(",");
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------


void Sensor::Putty_Euler_Angles(){ 

  Serial.print(this->Euler_angles_accelerometer(0));
  Serial.print(",");
  Serial.print(this->Euler_angles_accelerometer(1));
  Serial.print(",");
  Serial.print(this->Euler_angles_accelerometer(2));
  Serial.print(",");
  Serial.print(this->Euler_angles_gyroscope(0));
  Serial.print(",");
  Serial.print(this->Euler_angles_gyroscope(1));
  Serial.print(",");
  Serial.print(this->Euler_angles_gyroscope(2));
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------



void Sensor::Putty_Euler_Angles_Title(){

  Serial.print("Phi_acc,");
  Serial.print("Theta_acc,");
  Serial.print("Psi_acc,");
  Serial.print("Phi_gyr,");
  Serial.print("Theta_gyr,");
  Serial.print("Psi_gyr");

}