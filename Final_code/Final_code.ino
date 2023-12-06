// IMU pins
#define SDA_PIN  0 // IMU
#define SCL_PIN  1 // IMU
// Motor 1, 5 ,6
const int POT_PIN[3] = {35, 34, 39};
const int ENA[3] = {4, 0, 13};
const int IN1[3] = {2, 12, 27};
const int IN2[3] = {15, 14, 26};
const int PWM[3] = {0, 1, 2};

#define PI 3.141592
#define F 20
float dt = 1.0/F;
#define MILLION pow(10,6)

// Homemade headers
#include "Sensor.h"
#include "LKF.h"
#include "Pot_Control.h"
#include "Communications.h"

long int start;

Sensor sensor; // Sensor data
LKFType LKF; // Filter
POT Pot[3]; // motor 1, motor 5, motor 6 in that order.
PIController Controller[3]; // controller for motor 1, 5, and 6 in that order.
StaticJsonDocument<200> data_packet;
// Define Queues
static QueueHandle_t Queue_Measurements;
static QueueHandle_t Queue_MEKF;
static QueueHandle_t Queue_Estimation;
static QueueHandle_t Queue_Communications;

static const uint8_t msg_queue_len = 5;


// Data structures
typedef struct {
  float x;
  float y;
  float z;
} Vector3D;

typedef struct {
  float w;
  float x;
  float y;
  float z;
} Quaternion;

typedef struct {
  Vector3D magnet;
  Vector3D accel;
  Vector3D gyro;
} SensorData;

typedef struct {
  Quaternion quaternion;
  Vector3D torque;
} MEKFOutput;

typedef struct {
  Vector3D position;
} EstimationOutput;

// Global data
SensorData data;
MEKFOutput mekf_output;
EstimationOutput estimation_output;

// Tasks
void UpdateMeasurements(void *Parameters) {
  while (1) {
    // Update sensor measurements
    sensor.Update();
    // Extract sensor data and update global_sensor_data (adjust this part based on your sensor data structure)
    data.magnet.x = sensor.magn_meas(0);
    data.magnet.y = sensor.magn_meas(1);
    data.magnet.z = sensor.magn_meas(2);
    data.accel.x = sensor.accel_meas(0);
    data.accel.y = sensor.accel_meas(1);
    data.accel.z = sensor.accel_meas(2);
    data.gyro.x = sensor.gyro_meas(0);
    data.gyro.y = sensor.gyro_meas(1);
    data.gyro.z = sensor.gyro_meas(2);

    // Send data to MEKF and Estimation using Queues
    if (xQueueSend(Queue_Measurements, (void *) &data, 1) != pdTRUE){
      Serial.println("Data Queue full");
    }
    
    if (xQueueSend(Queue_Communication, (void *) &data, 1) != pdTRUE){
      Serial.println("Communication Queue full");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Use this for the frequency we want for the function
  }
}

void MEKF(void *Parameters) {
  while (1) {
   // Receive data from UpdateMeasurements
    xQueueReceive(Queue_Measurements, (void *) &data, 0)
    //MEKF code here
    mekf_output.quaternion.w = data.magnet.x + data.accel.y;
    mekf_output.torque.x = data.magnet.x + data.gyro.z;
    // Send result to Estimation
    if(xQueueSend(Queue_MEKF, (void *) &mekf_output, 1) != pdTRUE){
    Serial.println("MEKF Queue full");
  }
    if (xQueueSend(Queue_Communication, (void *) &mekf_output, 1) != pdTRUE){
      Serial.println("Communication Queue full");
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void Estimation(void *Parameters) {
 while (1) {
    xQueueReceive(Queue_MEKF, (void *) &mekf_output, 0)
    //Estimation code here



    if(xQueueSend(Queue_Estimation, (void *) &estimation_output, 10) != pdTRUE){
    Serial.println("Estimation Queue full");
   }
     if (xQueueSend(Queue_Communication, (void *) &estimation_output, 1) != pdTRUE){
      Serial.println("Communication Queue full");
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void Actuators(void *pvParameters) {
  float input[3];
  float output[3];
  float deadzone[3] = {1.9, 5.9, 5.9};

  while (1) {
    // Receive data from Estimation
    xQueueReceive(Queue_Estimation, (void *) &estimation_output, 0)

      // Reading the potentiometer and converting to position
      //input[0] = Pot[0].VoltageToPosition(Pot[0].PotVoltage());
      //input[1] = Pot[1].VoltageToPosition(Pot[1].PotVoltage());
      //input[2] = Pot[2].VoltageToPosition(Pot[2].PotVoltage());

      // Updating the controller
      //output[0] = Controller[0].Update(&estimation_output.position.x, input[0]);
      //output[1] = Controller[1].Update(&estimation_output.position.y, input[1]);
      //output[2] = Controller[2].Update(&estimation_output.position.z, input[2]);

      // Setting the motor voltage
      //Pot[0].setVoltageMotor(-output[0], Pot[0].DeadzoneDisable(deadzone[0]));
      //Pot[1].setVoltageMotor(-output[1], Pot[1].DeadzoneDisable(deadzone[1]));
      //Pot[2].setVoltageMotor(-output[2], Pot[2].DeadzoneDisable(deadzone[2]));

    vTaskDelay(1000/portTICK_PERIOD_MS);  // Use this for the frequency you want for the function
  }
}

void Communication(void *pvParameters) {
  while (1) {
    // Receive data from all tasks
    xQueueReceive(Queue_Communication, (void *) &data, 0);
    XBee.data_packet["Accelometer"][0] = data.accel.x;
    XBee.data_packet["Accelometer"][1] = data.accel.y;
    XBee.data_packet["Accelometer"][2] = data.accel.z;
    XBee.data_packet["Gyroscope"][0] = data.gyro.x;
    XBee.data_packet["Gyroscope"][1] = data.gyro.y;
    XBee.data_packet["Gyroscope"][2] = data.gyro.z;
    XBee.data_packet["Magnetometer"][0] = data.magnet.x;
    XBee.data_packet["Magnetometer"][1] = data.magnet.y;
    XBee.data_packet["Magnetometer"][2] = data.magnet.z;

    xQueueReceive(Queue_Communication, (void *) &mekf_output, 0);
    XBee.data_packet["Quaternion"][0] = mekf_output.quaternion.w;
    XBee.data_packet["Quaternion"][1] = mekf_output.quaternion.x;
    XBee.data_packet["Quaternion"][2] = mekf_output.quaternion.y;
    XBee.data_packet["Quaternion"][3] = mekf_output.quaternion.z;
    XBee.data_packet["Torque"][0] = mekf_output.torque.x;
    XBee.data_packet["Torque"][1] = mekf_output.torque.y;
    XBee.data_packet["Torque"][2] = mekf_output.torque.z;

    xQueueReceive(Queue_Communication, (void *) &estimation_output, 0);
    XBee.data_packet["Pot position"][0] = estimation_output.position.x;
    XBee.data_packet["Pot position"][1] = estimation_output.position.y;
    XBee.data_packet["Pot position"][2] = estimation_output.position.z;
    XBee.Send_Packet();
  }
}

void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  while(!Serial);
  sensor.Initialize(SDA_PIN,SCL_PIN);
  /* comment in when potentiometers attached
  Pot[0].Initialize(POT_PIN[0], ENA[0], IN1[0], IN2[0], PWM[0]);
  Pot[0].InitPotFunction(0.8041, 5.0581, 10.821, 0.5937);
  Controller[0].Initialize();
  Pot[1].Initialize(POT_PIN[1], ENA[1], IN1[1], IN2[1], PWM[1]);
  Pot[1].InitPotFunction(0.2521, 1.1326, 3.7706, 0.0422);
  Controller[1].Initialize();
  Pot[2].Initialize(POT_PIN[2], ENA[2], IN1[2], IN2[2], PWM[2]);
  Pot[2].InitPotFunction(0.814, 4.9125, 10.123, 0.9785);
  Controller[2].Initialize();*/
  XBee.Initialize(18, 17, 115200);

  // Create Queues
  Queue_Measurements = xQueueCreate(msg_queue_len, sizeof(SensorData));
  Queue_MEKF = xQueueCreate(msg_queue_len, sizeof(MEKFOutput));
  Queue_Estimation = xQueueCreate(msg_queue_len, sizeof(EstimationOutput));
  Queue_Communication = xQueueCreate(msg_queue_len, sizeof(SensorData));

  // Create tasks
  xTaskCreatePinnedToCore(UpdateMeasurements // The function attached
  , "UpdateMeasurements" // A name just for humans
  , 2048 // This stack size can be checked & adjusted by reading the Stack Highwater
  , NULL // Parameter to pass to function
  , 2 // Priority, with 2 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  , NULL //Task handle
  , 1); //Pinned Core
  xTaskCreatePinnedToCore(MEKF, "MEKF", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Estimation, "Estimation", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Actuators, "Actuators", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Communication, "Communication", 2048, NULL, 0, NULL, 1);

  //sensor.Calibrate_magnetometer();

}

void loop() {
  start = micros();
  // Unused in FreeRTOS-based applications

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
