const int SDA_PIN = 6;
const int SCL_PIN = 5;
#define PI 3.141592
#include <MPU9250_asukiaaa.h>
#include "WiFi_Settings.h"
//#include <Adafruit_BMP280.h>

//Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float alpha,phi,sigma;

#define F 20
#define TEST_TIME 10
DynamicJsonDocument doc_values_json(30000);
String data_serialised;




void upload_Data(String data_serialised);



void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(SDA_PIN, SCL_PIN);  
  mySensor.setWire(&Wire);

  //bme.begin();
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

void loop() {
  static JsonArray Ax_json = doc_values_json.createNestedArray("Ax");
  static JsonArray Ay_json = doc_values_json.createNestedArray("Ay");
  static JsonArray Az_json = doc_values_json.createNestedArray("Az");
  static JsonArray Gx_json = doc_values_json.createNestedArray("Gx");
  static JsonArray Gy_json = doc_values_json.createNestedArray("Gy");
  static JsonArray Gz_json = doc_values_json.createNestedArray("Gz");
  static JsonArray Mx_json = doc_values_json.createNestedArray("Mx");
  static JsonArray My_json = doc_values_json.createNestedArray("My");
  static JsonArray Mz_json = doc_values_json.createNestedArray("Mz");
  //Initializations for simulation
  static int i = 0;
  long int start = micros(); // Time control


  if (mySensor.accelUpdate() == 0) {
    Ax_json.add(mySensor.accelX());
    Ay_json.add(mySensor.accelY());
    Az_json.add(mySensor.accelZ());
  }
   if (mySensor.gyroUpdate() == 0) {
    Gx_json.add(mySensor.gyroX());
    Gy_json.add(mySensor.gyroY());
    Gz_json.add(mySensor.gyroZ());
  }

  if (mySensor.magUpdate() == 0) {
    Mx_json.add(mySensor.magX());
    My_json.add(mySensor.magY());
    Mz_json.add(mySensor.magZ());
  }


  if(((float)(micros()-start))/pow(10,6)>1.0/F){
    Serial.println("TIME EXCEEEEEEEEEEEEEEEEEEEEEEEEEDED");
  }
  if(i%F == 0){
    Serial.print("Time: ");
    Serial.println(i/F);
  }

  while(((float)(micros()-start))/pow(10,6)<1.0/F);
    i+=1;
    if(!(i< TEST_TIME*F)){
      // UploadData
      Serial.print("Size of each element:");
      Serial.println(Ax_json.size());
      serializeJson(doc_values_json,data_serialised);
      Serial.println("Json built. Sending...");
      upload_Data(data_serialised);
    }
  }


void upload_Data(String data_serialised){
  connect_to_wifi(ssid,password);
  delay(1000);

  Serial.print("Data upload to: ");
  Serial.println(serverName);


  POST_Data(data_serialised);
    while(true);

}

/**
void measure_Accel(){
  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    alpha = -asin(aX/aSqrt)*180/PI;
    if(aY == 0 && aZ == 0){
      phi = 0;
    }else if (aZ == 0)
      if(aY >0){
        phi = 90;
      }else{
        phi = -90;
      }
    
    else{
      phi = -atan(aY/aZ)*180/PI;
    }
  }
  Serial.print("time(s): ");
  Serial.print(t);
  Serial.print("\tPitch: "+String(alpha));
  Serial.print("\t Roll: "+String(phi));
  Serial.print("\taccelX: " + String(aX));
  Serial.print("\taccelY: " + String(aY));
  Serial.print("\taccelZ: " + String(aZ));
  Serial.print("\taccelSqrt: " + String(aSqrt));
}
**/