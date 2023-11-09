
#include "BLE.h"
#define BLE_BAUDRATE  115200
#define MAX_PACKETS 10
#ifdef ARDUINO
SoftwareSerial payloadBTSerial(2, 3); // RX, TX
#endif

struct Packet pkt_tx;
struct Packet pkt_rx;


#ifdef PAYLOAD
void ble_transmit(struct Packet toSend)
{
  pkt_tx.CFangleX_data = toSend.CFangleX_data;
  pkt_tx.gyroXvel_data = toSend.gyroXvel_data

  payloadBTSerial.write((byte ) & pkt_tx,sizeof(Packet));
}
#endif

#ifdef MAINPROC
void bleMain_setup()
{
  Serial6.begin(BLE_BAUDRATE);
}

//struct Packet ble_receive() {
//    const int PACKET_SIZE = sizeof(Packet);
//    int packets_processed = 0;
//    struct Packet most_recent_packet;
//
//    // While there's at least one packet's worth of data available
//    while (Serial6.available() >= PACKET_SIZE) {
//        // Read in the packet
//        Serial6.readBytes((byte) &most_recent_packet, PACKET_SIZE);
//        packets_processed++;
//    }
//
//    return most_recent_packet;
//}

struct Packet ble_receive()
{
  static byte count = 10;

  // Check the software serial buffer for data to read
  if((uint) Serial6.available() >= sizeof(Packet)) 
  {
    // Read in the appropriate number of bytes to fit our Packet
    Serial6.readBytes((byte *) & pkt_rx,sizeof(Packet));
    //while(Serial6.available() > 0)
      //Serial6.read();
  } 
  else
  {
    // If a disconnect happens, start transmitting
    if(count >= 10) 
    {
      count = 0;  // Reset counter
    }
    count++;
  }
  return pkt_rx;
}

#endif