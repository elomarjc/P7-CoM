#include "Communications.h"

void XBee::Initialize(int RX, int TX, int BaudRate){
    // Initialize the pins
	Serial1.setRX(RX);
	Serial1.setTx(TX);
	Serial1.begin(BaudRate);
}

void XBee::Send_Packet() {
  serializeJson(data_packet,serialised,200);
  Serial1.println(serialised);
  Serial.println(serialised);
}



