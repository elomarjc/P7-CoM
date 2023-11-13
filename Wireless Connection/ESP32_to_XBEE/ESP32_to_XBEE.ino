#include <ArduinoJson.h>
#include <HardwareSerial.h>
#define F 1


HardwareSerial XBEE_Serial(1);
StaticJsonDocument<200> data_packet;



void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  init_comms(18,17,115200);
  //Serial.println("Serial Txd is on pin: "+String(TX));
  //Serial.println("Serial Rxd is on pin: "+String(RX));
  data_packet["x"] = 2.45;
  data_packet["place"] = "Alabama";
}
void loop() { //Choose Serial1 or Serial2 as required
    long int start = micros();


    send_packet(data_packet);
    if(micros()-start > 1000000.0/F){
      Serial.print("TIME EXCEEEDED");
    }
    while(micros()-start < 1000000.0/F);
}



void init_comms(int RX, int TX,int baudRate){
  XBEE_Serial.begin(baudRate, SERIAL_8N1, RX, TX);
}
void send_packet(const JsonDocument& data){
 static char serialised[200];
  serializeJson(data_packet,serialised,200);
  XBEE_Serial.println(serialised);
  Serial.println(serialised);
}
