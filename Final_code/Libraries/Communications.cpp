#include "Communications.h"

void XBee::Initialize(int RX, int TX, int BaudRate){
    // Initialize the pins
    XBEE_Serial.begin(BaudRate, Serial_8N1, RX, TX);
}

void XBee::Send_Packet() {
  serializeJson(data_packet,serialised,200);
  XBEE_Serial.println(serialised);
  Serial.println(serialised);
}



