#ifndef COMMUNICATIONS
#define COMMUNICATIONS
#include <ArduinoJson.h>

//Declare in main: StaticJsonDocument<200> data_packet;


class XBee{
    public:
   // Must be executed in setup, initializes the XBee
    Initialize(int RX, int TX, int BaudRate);

    Send_Packet(data_packet);

    private:
    // Variables
	int RX;
	int TX;
	int baudRate;
	static char serialised[200];
};


#endif