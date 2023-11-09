#include <stdint.h>
#define PAYLOAD 1
#define ARDUINO 1
#ifdef ARDUINO

  #include "Arduino.h"
  #include <SoftwareSerial.h>

#endif
//https://www.mrswirlyeyes.com/tutorials/bluetooth_hm_10

struct Packet
{
  float CFangleX_data;
  float gyroXvel_data;
};

#ifdef PAYLOAD
void blePayload_setup();
void ble_transmit(struct Packet toSend);
#endif

#ifdef MAINPROC
void ble_receive();
#endif
void ble_setup();
