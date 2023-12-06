#ifndef COMMUNICATIONS
#define COMMUNICATIONS
#include <ArduinoJson.h>
#include <HardwareSerial.h>

class XBee{
    public:
    StaticJsonDocument<200> data_packet;
    // Must be executed in setup, initializes the XBee
    Initialize(int RX, int TX, int BaudRate);

    Send_Packet();

    private:
    // Variables
    HardwareSerial XBEE_Serial(1);
    static char serialised[200];
};


#endif