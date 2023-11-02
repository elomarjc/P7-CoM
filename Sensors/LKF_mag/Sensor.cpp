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



// ------------------------------------------------------------------------------------------------------------------------------------------------------

void Sensor::Update(){

  this->Update_Measurements();
  this->Filter_Measurements();
  this->Compute_axis();
  this->Compute_Euler_Angles();
}
void Sensor::Compute_axis(){
  axis_x_mag = Normalize(magn_meas_filtered);
  axis_z_accel = -Normalize(accel_meas_filtered);
  axis_y_mix = vector_product(axis_z_accel,axis_x_mag);
}

void Sensor::Update_Measurements(){
  if (gatherer.accelUpdate() == 0) {
    accel_meas = {gatherer.accelX(),gatherer.accelY(),gatherer.accelZ()};
    accel_meas -= accel_offset; 
  }
  

  
  if (gatherer.gyroUpdate() == 0) {
    gyro_meas = {-gatherer.gyroX(),-gatherer.gyroY(),-gatherer.gyroZ()}; // Minus make it follow convention: angles increase counter clockwise
    gyro_meas -= gyro_offset;
  }
  
  static bool first_round = true;
  if (gatherer.magUpdate() == 0) {
    magn_meas = {gatherer.magX(),gatherer.magY(),gatherer.magZ()};
    magn_meas = Normalize(magn_meas);

    if(first_round){
      first_round = false;
      magn_offset += magn_meas;
    }
    magn_meas -= magn_offset;
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