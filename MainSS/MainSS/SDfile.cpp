#include "core_pins.h"
/*
  SD card datalogger
 
 This example shows how to log data from three analog sensors
 to an SD card using the SD library.
 	
 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11, pin 7 on Teensy with audio board
 ** MISO - pin 12
 ** CLK - pin 13, pin 14 on Teensy with audio board
 ** CS - pin 4,  pin 10 on Teensy with audio board
 
 */
#include "SDfile.h"


//uint8_t counter = 0;
// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// Teensy audio board: pin 10
// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
// Wiz820+SD board: pin 4
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
const int chipSelect = BUILTIN_SDCARD;

void setup_SD()
{
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    /*
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
    */
  }
  Serial.println("card initialized.");
}

void writeToSD(struct Packet pkt_mainrx, uint8_t &bootmode, double &elapsed_time, long &startTime)
{
  String dataString = "Test Start! ";
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (bootmode == 0)
  {
    elapsed_time = 0;
    startTime = millis();
    dataString = "Starting Test! ";
    bootmode = 1;
    if (dataFile) 
    {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    }
    else 
    {
      // if the file isn't open, pop up an error:
      Serial.println("error opening datalog.txt");
    }
    //delay(100); // run at a reasonable not-too-fast speed
  }

  else 
  {
    if(dataFile)
    {
      /*
      dataFile.print("Sample: ");
      dataFile.print(String(sample));
      dataFile.print(" ");
      dataFile.print("AngleX Velocity: ");
      dataFile.print(String(pkt_mainrx.gyroXvel_data));
      dataFile.print(" ");
      dataFile.print("AngleX: ");
      dataFile.print(String(pkt_mainrx.CFangleX_data));
      dataFile.print(" ");
      */
      //sample++;
      //elapsed_time = pkt_mainrx.time_stamp + elapsed_time;
      //Serial.print("Elapsed Time SD: ");
      //Serial.println(elapsed_time);
      long time_lap = millis() - startTime;
      double time_lap_sec = (double) time_lap / 1000;
      elapsed_time = (double) (time_lap_sec / 1000) + elapsed_time;
      Serial.print("Start Time: ");
      Serial.print(startTime);
      Serial.print(" ");
      Serial.print("Time Lap (s): ");
      Serial.print(time_lap_sec);     
      Serial.print(" ");
      Serial.print("Elapsed time: ");
      Serial.println(elapsed_time);
      dataFile.print(elapsed_time);
      dataFile.print(" ");
      dataFile.print(pkt_mainrx.CFangleX_data);
      dataFile.print(" ");
      dataFile.println(pkt_mainrx.gyroXvel_data);

    }
    else 
    {
      // if the file isn't open, pop up an error:
      Serial.println("error opening datalog.txt");
    }
  }
  
}