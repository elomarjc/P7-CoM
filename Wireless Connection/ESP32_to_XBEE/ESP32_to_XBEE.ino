#include <ArduinoJson.h>
//#include <HardwareSerial.h>
#define F 1
int RX = 0;
int TX = 1;
//int xx = 1234;
//HardwareSerial XBEE_Serial();
StaticJsonDocument<200> data_packet;

void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //init_comms(0,1,115200);
  init_comms(RX, TX, 115200);
  //Serial.println("Serial Txd is on pin: "+String(TX));
  //Serial.println("Serial Rxd is on pin: "+String(RX));
  data_packet["x"] = 1234;
  data_packet["place"] = "alabama";
}
void loop() {  //Choose Serial1 or Serial2 as required
  long int start = micros();

  send_packet(data_packet);
  if (micros() - start > 1000000.0 / F) {
    Serial.print("TIME EXCEEEDED");
  }
  while (micros() - start < 1000000.0 / F);
}

/*void init_comms(int RX, int TX, int baudRate){
  XBEE_Serial.begin(baudRate, SERIAL_8N1, RX, TX); 
}
*/
void init_comms(int RX, int TX, int baudRate) {
  Serial1.setRX(RX);
  Serial1.setTX(TX);
  Serial1.begin(baudRate);
}

void send_packet(const JsonDocument& data) {
  static char serialised[200];
  serializeJson(data_packet, serialised, 200);
  //XBEE_Serial.println(serialised);
  Serial1.println(serialised);
  Serial.println(serialised);
}
