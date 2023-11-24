#ifndef SENSOR_H
#define SENSOR_H
#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <MPU9250_asukiaaa.h>
#include "Filter.h"

extern float dt;
class Sensor{
  public:
  // Variables ----------------------------------------------------------------------------
  BLA::Matrix<3,1,float> accel_meas = {0,0,0}; 
  BLA::Matrix<3,1,float> gyro_meas  = {0,0,0}; 
  BLA::Matrix<3,1,float> magn_meas  = {0,0,0};

  BLA::Matrix<3,1,float> accel_offset = {-0.0249,-0.0036,0.9617-1}; 
  BLA::Matrix<3,1,float> gyro_offset  = {-0.8630,-0.4062,-0.1584}; 
  BLA::Matrix<3,1,float> magn_soft_iron = {1,1,1};
  BLA::Matrix<3,1,float> magn_hard_iron = {0,0,0};

  BLA::Matrix<3,1,float> accel_meas_filtered = {0,0,0};
  BLA::Matrix<3,1,float> gyro_meas_filtered = {0,0,0};
  BLA::Matrix<3,1,float> magn_meas_filtered = {0,0,0};
  BLA::Matrix<3,1,float>  Euler_angles_accelerometer = {0,0,0};
  //BLA::Matrix<3,1,float>  Euler_angles_accelerometer_filtered = {0,0,0};
  
  BLA::Matrix<3,1,float>  Euler_angles_gyroscope= {0,0,0};


  BLA::Matrix<3,1,float> axis_x_mag = {0,0,0}; 
  BLA::Matrix<3,1,float> axis_y_mix = {0,0,0}; 
  BLA::Matrix<3,1,float> axis_z_accel = {0,0,0}; 


  // Funcitons ------------------------------------------------------------------------------
  // Must be executed in setup, connects to IMU and quickstarts it
  void Initialize(int pin_SDA,int pin_scl);

  // Updates the emasurements AND computes the euler angles
  void Update();
  // Prints Euler angles into Serial with Puttty formatting, both need the line change to be added outside of command
  void Putty_Euler_Angles_Title();
  void Putty_Euler_Angles();

  // Computes soft and hard iron effects
  void Calibrate_magnetometer();

  // Calculate angle for heading, assuming board is parallel to the ground
  float Heading();


  // Prints to serial for Plotter to read directly. Requires line change as well
  void Plotter_Euler_Angles();
  void Plotter_Gyro_Angles();
  void Plotter_Accel_Filtered();
  // ///////////////////////////////////////////PRIVATE/////////////////////////////////////////////

    // Dot product of 2 arrays
  float Dot_Product(float* vec1, float* vec2, int size);
  float Dot_Product(BLA::Matrix<3,1,float> vec1, BLA::Matrix<3,1,float> vec2);
    // Vector product form vec1 to vec2
  BLA::Matrix<3,1,float> vector_product(BLA::Matrix<3,1,float> vec1, BLA::Matrix<3,1,float> vec2);


  private:

  // Magnetic Declination for Aalborg
  float mag_dec1 = 4.11;

  MPU9250_asukiaaa gatherer;
  Filter accel_filter[3];

  // Updates the internal values with new raw measurements
  void Update_Measurements();
  // Performs the offset removal of the measurements
  void Offset_corrections();

  // Low-pass filter for measurements (currently only accelerometer)
  void Filter_Measurements();

  // Computes the euler angles using the normal methods.
  void Compute_Euler_Angles();

  // Computes the DCM vectors
  void Compute_axis();

  // Advances array of filter
  void Advance_Array(float*array,int length);

  //Calibrate magnetometer
  void get_Max_Min(float* samples, int size, float* max, float* min);

  // Normalizes vector
  BLA::Matrix<3,1,float> Normalize(BLA::Matrix<3,1,float>& vec_1);


};
// ###################################################################################################################################




#endif

