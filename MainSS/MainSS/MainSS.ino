//Developed by MEDEVAC Team's Codechief - Hoan Pham (mhpham23@vt.edu)
//MainProcessing SS code for Teensy 4.1
//#include "switch.h"
/*
#include <ezButton.h>
#include "HoistController.h"
#include "SDfile.h"
#include "LCD_screen.h"

//Define Macros:
#define ALGO_SWITCH 5
#define DEBOUNCE_TIME 50
#define BAUDRATE 115200
#define TEST_MODE   NONE
#define NONE        0
#define RAISING     1
#define LOWERING    2
#define ALGO        3

#define ANGLE_CONSTANT  2.8
#define THRESHOLD       2.0
#define VEL_CONSTANT    4.47
#define ROC_THRESHOLD   20
#define UP_SPEED        30
#define DOWN_SPEED      10


#define EPSILON         2 //Angle
#define ALPHA           4 //Angle Velocity


//Global Variables:
uint8_t bootmode = 0;
long long sample = 0;
double elapsed_time = 0.0;
long startTime = 0;
double current_speed = 0;
double new_speed = 0;

double last_pkt_time = 0;
struct Packet last_pktX;

struct Packet pkt_mainrx;
ezButton algoSwitch(ALGO_SWITCH);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);


//LANZEROTTI'S ALGO
void lanz_algo()
{
  float angle = pkt_mainrx.CFangle_data;
  float vel = pkt_mainrx.gyrovel_data;
  //TODO: Waiting for ryans
  if (angle < EPSILON && vel < -ALPHA)
  {
    set_lower_mode();
    ramp_speed(current_speed, DOWN_SPEED);
  }
  else if (angle < -EPSILON && vel >= -ALPHA)
  {
    set_raise_mode();
    ramp_speed(current_speed, UP_SPEED);
  }
  else if (angle > -EPSILON && vel > ALPHA)
  {
    set_lower_mode();
    ramp_speed(current_speed, DOWN_SPEED);
  }
  else if (angle > EPSILON && vel <= ALPHA)
  {
    set_raise_mode();
    ramp_speed(current_speed, UP_SPEED);    
  }
  else 
  {
    set_raise_mode();
    ramp_speed(current_speed, 0);
  }
  
} //END LANZ_ALGO()

bool accepted_ROC() {
  last_pktX = pkt_mainrx; // prev pkt
  pkt_mainrx = ble_receive(); // new pkt
  
  double current_time = millis();
  double time_difference = current_time - last_pkt_time; // elapsed time between packets

  double angleROC = (pkt_mainrx.CFangle_data - last_pktX.CFangle_data) / time_difference; // ROC of the angle

  last_pkt_time = current_time; // update last_pkt_time for the next iteration
  
  if (abs(pkt_mainrx.CFangle_data) < 20.0 && abs(angleROC) < ROC_THRESHOLD) {
    return true;
  }
  return false;
}

void ramp_speed(int current_speed, int new_speed) {
  int speed_difference = current_speed - new_speed;
  
   if (speed_difference >= 0) { // ramp down
      while (current_speed >= new_speed) {
        set_pwm_speed(current_speed--);
        delay(10);
      }
   }
   else { // ramp up
      while (current_speed <= new_speed) {
        set_pwm_speed(current_speed++);
        delay(10);
      }
   }
}

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

      
      lanz_algo();
      writeToSD(pkt_mainrx, bootmode, elapsed_time, startTime);

      pkt_mainrx = ble_receive();
      //writeToSD(pkt_mainrx, bootmode, sample);
      //LCD_printData(tft, pkt_mainrx.CFangleX_data, pkt_mainrx.gyroXvel_data);
      
    }
    else if (TEST_MODE == NONE)
    {
      //Speed 0, only read and log data
      stop();
      pkt_mainrx = ble_receive();

      writeToSD(pkt_mainrx, bootmode, elapsed_time, startTime);
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
      LCD_printData(tft, pkt_mainrx.CFangle_data, pkt_mainrx.gyrovel_data);
      set_lower_mode();
      set_pwm_speed(25);
      pkt_mainrx = ble_receive();
      writeToSD(pkt_mainrx, bootmode, elapsed_time, startTime);
      LCD_printData(tft, pkt_mainrx.CFangle_data, pkt_mainrx.gyrovel_data);      
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
    stop_all();
    LCD_printStabOff(tft);
    pkt_mainrx = ble_receive();
    LCD_printData(tft, pkt_mainrx.CFangle_data, pkt_mainrx.gyrovel_data);
    bootmode = 0;
  } //END MANUAL MODE



} //END VOID LOOP() */

//Developed by MEDEVAC Team's Codechief - Hoan Pham (mhpham23@vt.edu)
//MainProcessing SS code for Teensy 4.1
//#include "switch.h"
#include <ezButton.h>
#include "HoistController.h"
#include "SDfile.h"
#include "LCD_screen.h"

#define BAUDRATE 115200
#define ALGO_SWITCH 5
#define DEBOUNCE_TIME 50

#define ANGLE_CONSTANT  2.8
#define THRESHOLD       2.0
#define VEL_CONSTANT    2
#define ROC_THRESHOLD   20
#define UP_SPEED        30
#define DOWN_SPEED      10


#define EPSILON         1.6 //Angle
#define ALPHA           8 //Angle Velocity

//Global Variables:
uint8_t bootmode = 0;
long long sample = 0;
double elapsed_time = 0.0;
long startTime = 0;
double current_speeed = 0;
double new_speed = 0;

float last_x_angle = 0;
float last_x_vel = 0;


double last_pkt_time = 0;
struct Packet last_pktX;

struct Packet pkt_mainrx;
ezButton algoSwitch(ALGO_SWITCH);

int state = 1;
int num_oscillations = 0;

void lanz_algorithm() {
  float angle = pkt_mainrx.CFangleX_data;
  float vel = pkt_mainrx.gyroXvel_data;

  if ((angle < EPSILON && angle > -EPSILON) && (vel > ALPHA || vel < -ALPHA))
  {
    set_lower_mode();
    set_pwm_speed(DOWN_SPEED);
  }
  else if (angle < -EPSILON && ((vel >= -ALPHA && vel <= 0) || (vel <= ALPHA && vel >= 0)))
  {
    set_raise_mode();
    set_pwm_speed(UP_SPEED);
  }
  //      else if (angle < -EPSILON && vel <= ALPHA)
  //      {
  //        set_raise_mode();
  //        set_pwm_speed(UP_SPEED);
  //      }
  //      else if (angle > -EPSILON && vel > ALPHA)
  //      {
  //        set_lower_mode();
  //        set_pwm_speed(DOWN_SPEED);
  //      }
  //      else if (angle > EPSILON && vel <= ALPHA)
  //      {
  //        set_raise_mode();
  //        set_pwm_speed(UP_SPEED);
  //      }
  else if (angle > EPSILON && ((vel >= -ALPHA && vel <= 0) || (vel <= ALPHA && vel >= 0)))
  {
    set_raise_mode();
    set_pwm_speed(UP_SPEED);
  }
  else
  {
    //set_raise_mode();
    stop_all();
    set_pwm_speed(0);
    //Serial.println("stop all");
  }
}
// dummy
//        if (angle < EPSILON && vel < -ALPHA)
//    {
//      set_lower_mode();
//      set_pwm_speed(DOWN_SPEED);
//    }
//    else if (angle > EPSILON && vel >= -ALPHA)
//    {
//      set_raise_mode();
//      set_pwm_speed(UP_SPEED);
//    }
//    else if (angle < EPSILON && vel > ALPHA)
//    {
//      set_lower_mode();
//      set_pwm_speed(DOWN_SPEED);
//    }
//    else if (angle > EPSILON && vel <= ALPHA)
//    {
//      set_raise_mode();
//      set_pwm_speed(UP_SPEED);
//    }
//    else
//    {
//      //set_raise_mode();
//      set_pwm_speed(0);
//    }

void lqr() {
  double ROC = pkt_mainrx.gyroXvel_data;
  
  switch (state) {
    case 1:
      set_lower_mode();
      set_pwm_speed(-1.1 + (0.09 * num_oscillations));
    
      if (ROC > (12 - (2 * num_oscillations))) {
        state = 2;
      }
    case 2:
      stop_all();
      set_pwm_speed(0);
      
      if (ROC > (18 - (2.5 * num_oscillations))) {
        state = 3;
      }
    case 3:
      set_raise_mode();
      set_pwm_speed(.75 - (0.18 * num_oscillations));
      
      if (ROC < 0) {
        state = 4;
      }
    case 4:
      stop_all();
      set_pwm_speed(0);
      
      if (ROC < (-15 + (2 * num_oscillations))) {
        state = 1;
        num_oscillations = num_oscillations + 1;
      }
  }
}

void setup()
{
  Serial.begin(BAUDRATE);
  bleMain_setup();
  pkt_mainrx.CFangleX_data = 0.0;
  pkt_mainrx.gyroXvel_data = 0.0;
  setup_hoistController();
}

void loop()
{
  pkt_mainrx = ble_receive();
  pkt_mainrx.CFangleX_data = pkt_mainrx.CFangleX_data + 3;

  if (pkt_mainrx.CFangleX_data != last_x_angle) {
    Serial.println(pkt_mainrx.CFangleX_data);
  }

  //    if (pkt_mainrx.gyroXvel_data != last_x_vel) {
  //      //Serial.print(" ");
  //      Serial.print(pkt_mainrx.gyroXvel_data);
  //    }

  if ((pkt_mainrx.CFangleX_data != last_x_angle) || (pkt_mainrx.gyroXvel_data != last_x_vel)) {
    lanz_algorithm();
    //lqr();
  }

  last_x_angle = pkt_mainrx.CFangleX_data;
  last_x_vel = pkt_mainrx.gyroXvel_data;

  delay(5);

}
