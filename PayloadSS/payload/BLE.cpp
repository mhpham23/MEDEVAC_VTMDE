#include "BLE.h"

//https://www.mrswirlyeyes.com/tutorials/bluetooth_hm_10
#define BLE_BAUDRATE  115200
#ifdef ARDUINO
SoftwareSerial payloadBTSerial(2, 3); // RX, TX
#endif

struct Packet pkt_tx;
struct Packet pkt_rx;

#ifdef PAYLOAD
void blePayload_setup()
{
  payloadBTSerial.begin(BLE_BAUDRATE);
}
void ble_transmit(struct Packet toSend)
{
  pkt_tx.CFangleX_data = toSend.CFangleX_data;
  pkt_tx.gyroXvel_data = toSend.gyroXvel_data;

  payloadBTSerial.write((byte *) & pkt_tx,sizeof(Packet));
  /*
  Serial.print("TX: ");
  //Serial.print(" ");
  //Serial.print(pkt_tx.gyroXvel_data);
  Serial.print(" ");
  Serial.print(pkt_tx.CFangle_data);
  Serial.print(" ");
  Serial.println(pkt_tx.gyrovel_data);
  */
}
#endif

#ifdef MAINPROC
void ble_receive()
{
// Counting variable to fix a lost connection
  static byte count = 10;
  
  // Check the software serial buffer for data to read
  if(Serial1.available() >= sizeof(Packet)) 
  {
    // Read in the appropriate number of bytes to fit our Packet
    Serial1.readBytes((byte *) & pkt_rx,sizeof(Packet));
    
    // Print the Packet contents
    Serial.print("RX: ");
    Serial.print(pkt_rx.CFangleX_data);
    Serial.print(" ");
    Serial.println(pkt_rx.gyroXvel_data);
    
    // Flush the serial buffer
    while(Serial1.available() > 0)
      Serial1.read();   
    
    // Transmit data via bluetooth
    //ble_transmit();
  } 
  else 
  {
    // If a disconnect happens, start transmitting
    if(count >= 10) 
    {
      count = 0;  // Reset counter
      // Transmit to revive process
      //ble_transmit();
    }
    count++;
  }
}
#endif