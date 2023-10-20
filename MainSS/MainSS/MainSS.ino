//Developed by MEDEVAC Team's Codechief - Hoan Pham (mhpham23@vt.edu)
//MainProcessing SS code for Teensy 4.1
//#include "switch.h"
#include <ezButton.h>
#include "HoistController.h"
#include "SDfile.h"
#include "LCD_screen.h"

//Define Macros:
#define ALGO_SWITCH 5
#define DEBOUNCE_TIME 50
#define BAUDRATE 115200
#define TEST_MODE   RAISING
#define NONE        0
#define RAISING     1
#define LOWERING    2
#define ALGO        3

#define ANGLE_CONSTANT  2.8
#define THRESHOLD       2.0
#define VEL_CONSTANT    4.47
#define DOWN_SPEED      
#define UP_SPEED

//Global Variables:
uint8_t bootmode = 0;
long long sample = 0;
double elapsed_time = 0.0;
long startTime = 0;

struct Packet pkt_mainrx;
ezButton algoSwitch(ALGO_SWITCH);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

//LANZEROTTI'S ALGO
void lanz_algo()
{
  //TODO: Waiting for ryans
} //END LANZ_ALGO()

void test_proc()
{
  dbg_print();
  Serial.println("Test Proc!");
  pkt_mainrx = ble_receive();
  Serial.println(pkt_mainrx.CFangle_data);
  if ((pkt_mainrx.CFangle_data > 10.0) || (pkt_mainrx.CFangle_data < -10.0))
  {
    setup_hoistController();
    set_raise_mode();
    set_pwm_speed(20);
    Serial.println("RAISING!");

  }
  else if ((pkt_mainrx.CFangle_data > -2.0) && (pkt_mainrx.CFangle_data < 2.0))
  {
    setup_hoistController();
    set_lower_mode();
    set_pwm_speed(10);
    Serial.println("LOWERING!");
  }
  else
  {
    off();
    Serial.println("RELAY OFF!");
  }
}

void test_proc2()
{
  dbg_print();
  Serial.println("Test Proc 2!");
  pkt_mainrx = ble_receive();
  Serial.println(pkt_mainrx.CFangle_data);
  if ((pkt_mainrx.CFangle_data > -2.0) && (pkt_mainrx.CFangle_data < 2.0))
  {
    setup_hoistController();
    //set_lower_mode();
    set_raise_mode();
    set_pwm_speed(5);
    Serial.println("DOWN!");
  }
  else if ((pkt_mainrx.gyrovel_data > -12.0) && (pkt_mainrx.gyrovel_data < 12.0))
  {
    setup_hoistController();
    set_raise_mode();
    set_pwm_speed(50);
    Serial.println("UP!");
  }
  else
  {
    //off();
    setup_hoistController();
    set_raise_mode();
    set_pwm_speed(20);
    Serial.println("RELAY OFF!");
  }
}
void dbg_print()
{
  Serial.print("Main RX: ");
  Serial.print(pkt_mainrx.CFangle_data);
  Serial.print(" ");
  Serial.println(pkt_mainrx.gyrovel_data);
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(BAUDRATE);
  //setup_switch(algoSwitch);
  algoSwitch.setDebounceTime(50);
  setup_hoistController();
  bleMain_setup();
  setup_SD();
  setup_LCD(tft);
  pkt_mainrx.CFangleX_data = 0.0;
  pkt_mainrx.gyroXvel_data = 0.0;
  pkt_mainrx.CFangle_data = 0.0;
  pkt_mainrx.gyrovel_data = 0.0;
  Serial.println("SETUP COMPLETED!");
} //END VOID SETUP()

void loop() 
{
  // put your main code here, to run repeatedly:
  //algoSwitchInit(algoSwitch);
  algoSwitch.loop();
  LCD_printSensorMode(tft);
  LCD_printBLEstatus(tft);
  Serial.println("Finish Printing Init!");
  int switchState = algoSwitch.getState();
  //AUTOMATIC MODE ON
  if(switchState == LOW)
  //if(algoSwitch.isPressed())
  {
    Serial.println("SWITCH IS ON");
    setup_hoistController();
    //writeToSD(pkt_mainrx, bootmode, sample);
    pkt_mainrx = ble_receive();
    //writeToSD(pkt_mainrx, bootmode, sample);
    Serial.println("Checking Test Mode");
    if(TEST_MODE == ALGO)
    {
      //ALGO
      Serial.println("ALGO MODE");
      //LCD_printStabOn(tft);
      pkt_mainrx = ble_receive();
      Serial.println("Receiving BLE");
      //writeToSD(pkt_mainrx, bootmode, sample);
      //LCD_printData(tft, pkt_mainrx.CFangleX_data, pkt_mainrx.gyroXvel_data);
      test_proc2();
      pkt_mainrx = ble_receive();
      //writeToSD(pkt_mainrx, bootmode, sample);
      //LCD_printData(tft, pkt_mainrx.CFangleX_data, pkt_mainrx.gyroXvel_data);
      test_proc2();
    }
    else if (TEST_MODE == NONE)
    {
      //Speed 0, only read and log data
      stop();
      pkt_mainrx = ble_receive();
      //writeToSD(pkt_mainrx, bootmode, sample);
      //LCD_printData(tft, pkt_mainrx.CFangleX_data, pkt_mainrx.gyroXvel_data);
    }
    else if (TEST_MODE == RAISING) 
    {
      //LCD_printStabOff(tft);

      //Pulling up speed 25% and log data
      pkt_mainrx = ble_receive();
      dbg_print();
      Serial.println("Receiving BLE");
      writeToSD(pkt_mainrx, bootmode, elapsed_time, startTime);
      //LCD_printData(tft, pkt_mainrx.CFangleX_data, pkt_mainrx.gyroXvel_data);
      setup_hoistController();
      //set_raise_mode();
      set_lower_mode();
      set_pwm_speed(20);
      Serial.println("outputting 90%");
      pkt_mainrx = ble_receive();
      dbg_print();
      writeToSD(pkt_mainrx, bootmode, elapsed_time, startTime);
      //LCD_printData(tft, pkt_mainrx.CFangleX_data, pkt_mainrx.gyroXvel_data);

    }
    else if (TEST_MODE == LOWERING)
    {
      //Lower the hoist speed 25% and log data
      LCD_printStabOff(tft);
      //Pulling up speed 25% and log data
      pkt_mainrx = ble_receive();
      writeToSD(pkt_mainrx, bootmode, elapsed_time, startTime);
      LCD_printData(tft, pkt_mainrx.CFangleX_data, pkt_mainrx.gyroXvel_data);
      set_lower_mode();
      set_pwm_speed(25);
      pkt_mainrx = ble_receive();
      writeToSD(pkt_mainrx, bootmode, elapsed_time, startTime);
      LCD_printData(tft, pkt_mainrx.CFangleX_data, pkt_mainrx.gyroXvel_data);      
    }
    else 
    {
      //STOP ALL
      stop_all();
      Serial.println("ERROR! WRONG TEST MODE!");
    }
  } //END AUTOMATIC ON
  
  //MANUAL MODE
  else
  {
    Serial.println("Switch Off!");
    //Turning hoist controller off
    bootmode = 0;
    stop_all();
    LCD_printStabOff(tft);
    pkt_mainrx = ble_receive();
    LCD_printData(tft, pkt_mainrx.CFangleX_data, pkt_mainrx.gyroXvel_data);
  } //END MANUAL MODE



} //END VOID LOOP()
