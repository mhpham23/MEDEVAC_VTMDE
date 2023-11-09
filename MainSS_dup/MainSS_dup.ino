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
#define UP_SPEED        80
#define DOWN_SPEED      20


#define EPSILON         5 //Angle
#define ALPHA           12 //Angle Velocity

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
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

int state = 1;
int num_oscillations = 0;

void lanz_algorithm() {
  float angle = pkt_mainrx.CFangleX_data;
  float vel = pkt_mainrx.gyroXvel_data;

  if ((angle < EPSILON && angle > -EPSILON) && (vel > ALPHA || vel < -ALPHA))
  {
    set_lower_mode();
    set_pwm_speed(DOWN_SPEED);
    //delay(2);
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
    stop();
    set_pwm_speed(0);
    //Serial.println("stop all");
    //delay(2);
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
  setup_LCD(tft);
  LCD_printStabOn(tft);
  LCD_printStabOff(tft);
  LCD_printSensorMode(tft);
  LCD_printBLEstatus(tft);
}

void loop()
{
  pkt_mainrx = ble_receive();
  pkt_mainrx.CFangleX_data = pkt_mainrx.CFangleX_data + 3;
  pkt_mainrx.gyroXvel_data = pkt_mainrx.gyroXvel_data;

  if (pkt_mainrx.CFangleX_data != last_x_angle) {
    Serial.print(pkt_mainrx.CFangleX_data);
    Serial.print(",");
    Serial.println(pkt_mainrx.gyroXvel_data);
  }

//  if (pkt_mainrx.gyroXvel_data != last_x_vel) {
//    //Serial.print(" ");
//    Serial.println(pkt_mainrx.gyroXvel_data);
//  }

  if ((pkt_mainrx.CFangleX_data != last_x_angle) || (pkt_mainrx.gyroXvel_data != last_x_vel)) {
    lanz_algorithm();
    //lqr();
  }

  last_x_angle = pkt_mainrx.CFangleX_data;
  last_x_vel = pkt_mainrx.gyroXvel_data;

  delay(5);

}
